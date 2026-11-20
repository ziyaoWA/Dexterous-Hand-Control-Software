"""High-81339 MIT trajectory, register control, and RS485 upgrade console."""

from collections import deque
import csv
from itertools import islice
import math
from pathlib import Path
import queue
from threading import Thread
import tkinter as tk
from tkinter import filedialog, messagebox, ttk

from console_session import ConsoleSession
from flash_page import FlashPage
from monitor_plot import MonitorPlot
from register_control import RegisterControlMixin
from register_pages import RegisterPages
from trajectory_core import plan_trajectory, preview_trajectory
from trajectory_plot import TrajectoryPlot
from trajectory_wire import iter_setpoints, parse_exchange
from tuning_core import supports_mit


class TrajectorySession(RegisterControlMixin, ConsoleSession):
    """Forward verified exchanges and decoded samples to the window."""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.stream_request = None

    def connect(self, payload):
        super().connect(payload)
        if self.connection_info['baudrate'] == 3000000:
            self.read_parameters()

    def log(self, event, **fields):
        super().log(event, **fields)
        if event == 'mit_trajectory_tx':
            self.stream_request = fields['tx']
        elif event == 'mit_trajectory_ack':
            self._publish_exchange(fields['rx'])
        elif event == 'mit_trajectory_started':
            self.send('plot_plan', {'id': fields['id'], 'preview': fields['preview']})
        elif event == 'parameter_readback':
            self._publish_words(0, fields['f0'].copy())
            self._publish_words(0x100, fields['f1'].copy())
        elif event == 'status' and self.mit_trajectory is None:
            values = fields['values']
            self._publish_words(0x6000, values[:29])
            self._publish_words(0x6025, values[37:46])
        elif event == 'status' and self.mit_trajectory is not None:
            sample = fields['decoded']
            trajectory = self.mit_trajectory
            if sample['trial'] == trajectory['id'] and (sample['position_flags'] & 0x37) == 1:
                self.send('plot_feedback', {
                    'id': sample['trial'],
                    'elapsed': sample['monotonic'] - trajectory['started'],
                    'position_deg': sample['position_actual_deg'],
                    'speed_rpm': sample['speed_actual_rpm'],
                })

    def _publish_exchange(self, reply='', error=None):
        if self.stream_request is not None:
            self.send('wire', {'tx': self.stream_request, 'rx': reply, 'error': error})
            self.stream_request = None

    def _mit_poll_exchange(self):
        try:
            return super()._mit_poll_exchange()
        except OSError as error:
            pending = self.mit_trajectory['pending']
            reply = pending['buffer'].hex(' ').upper()
            self.log('trajectory_reply_rejected', tx=self.stream_request, rx=reply, error=str(error))
            self._publish_exchange(reply, error=str(error))
            raise

    def stop(self, reason='操作者请求停止'):
        self._publish_exchange(error='收到停止请求，事务未完成')
        super().stop(reason)
        if self.manual_run:
            self.manual_run = False
            self.send('manual_state', False)

    def disconnect(self, unused=None):
        if self.manual_run and self.rtu is not None:
            self.stop('断开连接前停止 C0 运行')
        super().disconnect(unused)


def table(parent, columns, height):
    frame = ttk.Frame(parent)
    frame.pack(fill='both', expand=True)
    view = ttk.Treeview(frame, columns=[name for name, _ in columns], show='headings', height=height)
    for name, width in columns:
        view.heading(name, text=name)
        view.column(name, width=width, minwidth=60, stretch=True)
    ybar = ttk.Scrollbar(frame, orient='vertical', command=view.yview)
    xbar = ttk.Scrollbar(frame, orient='horizontal', command=view.xview)
    view.configure(yscrollcommand=ybar.set, xscrollcommand=xbar.set)
    view.grid(row=0, column=0, sticky='nsew')
    ybar.grid(row=0, column=1, sticky='ns')
    xbar.grid(row=1, column=0, sticky='ew')
    frame.rowconfigure(0, weight=1)
    frame.columnconfigure(0, weight=1)
    return view


class TrajectoryHost:
    PREVIEW_ROWS = 200
    WIRE_ROWS = 500

    def __init__(self, root):
        self.root = root
        self.events = queue.Queue()
        self.session = None
        self.thread = None
        self.link = 'closed'
        self.motion = 'idle'
        self.latest_state = None
        self.mit_supported = False
        self.firmware_marker = None
        self.register_busy = False
        self.manual_running = False
        self.flash_busy = False
        self.monitor_active = False
        self.monitor_interval = 200
        self.monitor_timer = None
        self.monitor_pending = None
        self.monitor_run = 0
        self.monitor_source = ''
        self.closing = False
        self.plot_trial = None
        self.wire_ids = deque()
        self.wire_data = {}

        root.title('High-81339 · MIT 轨迹与通讯 · 曲线版')
        root.geometry('1160x780')
        root.minsize(900, 620)
        root.protocol('WM_DELETE_WINDOW', self.close)
        style = ttk.Style()
        style.configure('.', font=('Microsoft YaHei UI', 10))
        style.configure('Title.TLabel', font=('Microsoft YaHei UI', 17, 'bold'))
        style.configure('Treeview', rowheight=25)
        main = ttk.Frame(root, padding=14)
        main.pack(fill='both', expand=True)
        ttk.Label(main, text='MIT 轨迹与通讯', style='Title.TLabel').pack(anchor='w')
        self.build_connection(main)

        self.tabs = ttk.Notebook(main)
        self.tabs.pack(fill='both', expand=True, pady=(8, 0))
        path_page = ttk.Frame(self.tabs, padding=10)
        self.plot_page = TrajectoryPlot(self.tabs)
        reply_page = ttk.Frame(self.tabs, padding=10)
        self.tabs.add(path_page, text='轨迹发包')
        self.tabs.add(self.plot_page, text='速度与位置')
        self.tabs.add(reply_page, text='回包解析')
        self.build_path_page(path_page)
        self.build_reply_page(reply_page)
        self.register_pages = RegisterPages(self.tabs, self.request_register, self.stop, self.note_message)
        monitor_tabs = self.register_pages.monitor_tabs
        self.monitor_plot = MonitorPlot(monitor_tabs, self.start_monitor, self.stop_monitor)
        monitor_tabs.add(self.monitor_plot, text='实时曲线')
        self.flash_page = FlashPage(self.tabs, self.request_flash, self.open_loader, self.cancel_flash)
        self.tabs.add(self.flash_page, text='485 烧录')
        self.note = tk.StringVar(value='生成轨迹帧后可查看速度与位置曲线，也可粘贴一组请求与回包进行解析。')
        ttk.Label(main, textvariable=self.note, wraplength=1100).pack(fill='x', pady=(8, 0))
        self.refresh_ports()
        self.refresh_controls()
        root.after(30, self.pump)

    def build_connection(self, parent):
        row = ttk.Frame(parent)
        row.pack(fill='x', pady=(8, 0))
        self.port = tk.StringVar()
        self.node = tk.StringVar(value='1')
        ttk.Label(row, text='串口').pack(side='left')
        self.port_box = ttk.Combobox(row, textvariable=self.port, width=12)
        self.port_box.pack(side='left', padx=(5, 8))
        self.refresh_button = ttk.Button(row, text='刷新', command=self.refresh_ports)
        self.refresh_button.pack(side='left', padx=(0, 12))
        ttk.Label(row, text='站号').pack(side='left')
        self.node_box = ttk.Combobox(row, textvariable=self.node, values=tuple(range(1, 6)),
                                     state='readonly', width=4)
        self.node_box.pack(side='left', padx=5)
        self.serial_format = tk.StringVar(value='3 Mbps · 8N1')
        ttk.Label(row, textvariable=self.serial_format).pack(side='left', padx=8)
        self.connect_button = ttk.Button(row, text='连接并读状态', command=self.connect)
        self.connect_button.pack(side='left', padx=4)
        self.read_button = ttk.Button(row, text='读取状态', command=lambda: self.submit('read_once'))
        self.read_button.pack(side='left', padx=4)
        self.disconnect_button = ttk.Button(row, text='断开', command=self.disconnect)
        self.disconnect_button.pack(side='left', padx=4)
        self.stop_button = tk.Button(row, text='停止', command=self.stop,
                                     background='#b82735', foreground='white', padx=18)
        self.stop_button.pack(side='right')
        self.status = tk.StringVar(value='离线')
        ttk.Label(parent, textvariable=self.status, wraplength=1100).pack(anchor='w', pady=(6, 0))

    def build_path_page(self, parent):
        form = ttk.Frame(parent)
        form.pack(fill='x')
        options = (
            ('start_deg', '离线起点 °', '0'),
            ('target_deg', '绝对目标 °', ''),
            ('max_motor_rpm', '电机端最高 rpm', '500'),
            ('accel_seconds', '加速 s', '0.5'),
            ('decel_seconds', '减速 s', '1'),
            ('period_ms', '更新间隔 ms', '20'),
            ('hold_seconds', '终点保持 s', '5'),
        )
        self.inputs = {}
        for column, (key, label, value) in enumerate(options):
            form.columnconfigure(column, weight=1)
            ttk.Label(form, text=label).grid(row=0, column=column, sticky='w', padx=3)
            variable = self.inputs[key] = tk.StringVar(value=value)
            entry = ttk.Entry(form, textvariable=variable, width=12)
            entry.grid(row=1, column=column, sticky='ew', padx=3)
            if key == 'start_deg':
                self.start_entry = entry
        actions = ttk.Frame(parent)
        actions.pack(fill='x', pady=8)
        self.generate_button = ttk.Button(actions, text='生成轨迹帧', command=self.generate)
        self.generate_button.pack(side='left', padx=3)
        self.export_button = ttk.Button(actions, text='导出全部帧 CSV', command=self.export_frames)
        self.export_button.pack(side='left', padx=3)
        self.run_button = ttk.Button(actions, text='发送单次轨迹…', command=self.start)
        self.run_button.pack(side='left', padx=3)
        self.plan_text = tk.StringVar(value='减速比 52.69。连接后使用读回的实际位置为起点。')
        ttk.Label(parent, textvariable=self.plan_text, wraplength=1080).pack(anchor='w', pady=(0, 4))
        self.motion_text = tk.StringVar()
        ttk.Label(parent, textvariable=self.motion_text, foreground='#145582').pack(anchor='w', pady=(0, 4))
        self.frames = table(parent, [('时间 s', 82), ('输出角度 °', 105), ('电机 rpm', 95),
                                     ('速度帧 F0-01', 300), ('位置帧 F0-02', 300)], 13)

    def build_reply_page(self, parent):
        form = ttk.Frame(parent)
        form.pack(fill='x', pady=(0, 6))
        self.tx, self.rx = tk.StringVar(), tk.StringVar()
        for number, (label, variable) in enumerate((('请求 HEX', self.tx), ('回包 HEX', self.rx))):
            ttk.Label(form, text=label).grid(row=number, column=0, sticky='w', padx=(0, 6), pady=3)
            ttk.Entry(form, textvariable=variable, font=('Consolas', 10)).grid(
                row=number, column=1, sticky='ew', pady=3)
        form.columnconfigure(1, weight=1)
        ttk.Button(form, text='解析', command=self.parse_input).grid(row=0, column=2, rowspan=2, padx=8)
        self.reply_summary = tk.StringVar(value='粘贴单条请求与回包，或从下方收发记录选择一行。')
        ttk.Label(parent, textvariable=self.reply_summary, wraplength=1080).pack(anchor='w', pady=(0, 4))
        self.reply_fields = table(parent, [('地址', 100), ('寄存器字段', 430), ('原始字', 118),
                                          ('解析值', 260)], 7)
        ttk.Label(parent, text='最近 500 条收发 · 单击查看完整帧').pack(anchor='w', pady=(9, 4))
        self.wire = table(parent, [('请求 HEX', 270), ('回包 HEX', 270), ('结果', 380)], 8)
        self.wire.bind('<<TreeviewSelect>>', self.select_wire)

    def refresh_ports(self):
        try:
            from serial.tools import list_ports
            ports = [port.device for port in list_ports.comports()]
        except (ImportError, OSError):
            ports = []
        self.port_box.configure(values=ports)
        if len(ports) == 1 and not self.port.get():
            self.port.set(ports[0])

    def refresh_controls(self):
        controls_available = not self.closing and not self.flash_busy
        offline = self.link == 'closed' and controls_available
        ready = self.link == 'ready' and controls_available
        loader = self.link == 'loader' and controls_available
        idle = self.motion == 'idle'
        self.port_box.configure(state='normal' if offline else 'disabled')
        self.start_entry.configure(state='normal' if offline else 'disabled')
        self.node_box.configure(state='readonly' if offline else 'disabled')
        for widget, enabled in ((self.refresh_button, offline), (self.connect_button, offline),
                                (self.read_button, ready and idle and not self.register_busy),
                                (self.disconnect_button, (ready or loader) and self.motion != 'stopping'),
                                (self.run_button, ready and idle and self.mit_supported
                                 and not self.register_busy and not self.manual_running),
                                (self.stop_button, ready and self.motion != 'stopping'),
                                (self.generate_button, offline or ready and idle),
                                (self.export_button, offline or ready and idle)):
            widget.configure(state='normal' if enabled else 'disabled')
        self.register_pages.set_state(ready, self.register_busy, not idle, self.firmware_marker)
        self.monitor_plot.set_state(ready and idle and not self.register_busy, self.monitor_active)
        self.flash_page.set_state(self.link, self.flash_available(), self.flash_busy,
                                  self.port.get(), self.node.get())

    def start_monitor(self, interval_ms):
        if (self.link != 'ready' or self.motion != 'idle' or self.register_busy
                or self.flash_busy or self.closing or self.monitor_active):
            return
        self.monitor_interval = interval_ms
        self.monitor_run += 1
        self.monitor_active = True
        self.monitor_source = f'{self.port.get()} · ID{self.node.get()}'
        self.monitor_plot.clear_samples()
        self.monitor_plot.set_status(self.monitor_source + ' · 正在采样')
        self.refresh_controls()
        self.schedule_monitor(0)

    def stop_monitor(self, message='采样已停止，可切换曲线查看本次记录。'):
        if not self.monitor_active:
            return
        self.monitor_active = False
        if self.monitor_timer is not None:
            self.root.after_cancel(self.monitor_timer)
            self.monitor_timer = None
        self.monitor_plot.set_status(self.monitor_source + ' · ' + message)
        self.refresh_controls()

    def schedule_monitor(self, delay=None):
        if self.monitor_active and self.monitor_timer is None:
            try:
                self.monitor_timer = self.root.after(
                    self.monitor_interval if delay is None else delay, self.poll_monitor)
            except tk.TclError:
                self.stop_monitor('采样间隔无法由系统计时，请输入较小的毫秒值。')

    def poll_monitor(self):
        self.monitor_timer = None
        if not self.monitor_active:
            return
        if self.link != 'ready' or self.flash_busy or self.closing:
            self.stop_monitor()
        elif self.register_busy or self.motion != 'idle' or self.monitor_pending is not None:
            self.schedule_monitor()
        else:
            self.monitor_pending = self.monitor_run
            self.submit('sample_monitor', self.monitor_pending)

    def monitor_result(self, data):
        if data['request'] != self.monitor_pending:
            return
        self.monitor_pending = None
        if not self.monitor_active:
            return
        if data['request'] == self.monitor_run:
            if 'error' in data:
                self.stop_monitor('采样已停止。' + data['error'])
                return
            if 'values' in data:
                self.monitor_plot.add_sample(data['time'], data['values'])
        self.schedule_monitor()

    def flash_available(self):
        return (not self.closing and self.motion == 'idle'
                and not self.register_busy and not self.manual_running)

    def open_loader(self):
        self.connect(baudrate=1000000)

    def request_flash(self, action, payload):
        links = {
            'firmware_validate': ('closed', 'ready', 'loader'),
            'firmware_enter': ('ready',),
            'firmware_info': ('loader',),
            'firmware_dry_run': ('ready', 'loader'),
            'firmware_upgrade': ('ready', 'loader'),
        }
        if self.flash_busy or not self.flash_available() or self.link not in links[action]:
            return
        if self.ensure_session():
            self.flash_busy = self.session.firmware_pending = True
            self.stop_monitor('进入烧录页操作，已停止采样。')
            self.monitor_pending = None
            self.refresh_controls()
            self.submit(action, payload)

    def cancel_flash(self):
        if self.flash_busy:
            self.session.firmware_cancel.set()

    def note_message(self, message):
        self.note.set(message)

    def request_register(self, payload):
        if (self.link != 'ready' or self.motion != 'idle' or self.register_busy
                or self.flash_busy or self.closing):
            return
        self.register_busy = True
        self.refresh_controls()
        self.submit('control_registers', payload)

    def ensure_session(self):
        if self.session is None:
            try:
                self.session = TrajectorySession(
                    lambda event, value: self.events.put((event, value)),
                    root=Path(__file__).resolve().parent / 'trajectory_runs')
            except OSError as error:
                self.note.set(f'无法创建收发日志：{error}')
                return False
            self.thread = Thread(target=self.session.worker, daemon=True)
            self.thread.start()
        return True

    def connect(self, baudrate=3000000):
        if self.link != 'closed' or self.flash_busy or self.closing:
            return
        port = self.port.get().strip()
        if not port:
            self.note.set('请选择或填写串口，例如 COM17。')
            return
        if not self.ensure_session():
            return
        self.link = 'opening'
        self.serial_format.set(f'{baudrate // 1000000} Mbps · 8N1')
        self.status.set(f'正在连接 {port} · 站号 {self.node.get()}')
        self.refresh_controls()
        self.submit('connect', {'port': port, 'node': int(self.node.get()), 'baudrate': baudrate})

    def submit(self, action, payload=None):
        self.session.jobs.put((action, payload))

    def disconnect(self):
        self.link = 'disconnecting'
        self.stop_monitor('正在断开串口，已停止采样。')
        self.refresh_controls()
        self.submit('disconnect')

    def stop(self):
        self.motion = 'stopping'
        self.note.set('正在发送停止并读取停机状态…')
        self.refresh_controls()
        self.session.urgent_stop.set()

    def plan_input(self):
        payload = {key: self.inputs[key].get() for key in (
            'target_deg', 'max_motor_rpm', 'accel_seconds', 'decel_seconds', 'hold_seconds')}
        payload['period_seconds'] = float(self.inputs['period_ms'].get()) / 1000
        start = self.latest_state['position_actual_deg'] if self.latest_state else self.inputs['start_deg'].get()
        return plan_trajectory(start, payload), int(self.node.get()), payload

    def generate(self, update_plot=True):
        try:
            plan, node, payload = self.plan_input()
        except ValueError as error:
            self.frames.delete(*self.frames.get_children())
            self.plan_text.set(str(error))
            return None
        self.frames.delete(*self.frames.get_children())
        count = math.ceil(plan.duration / plan.period_seconds) + 1
        for t, p, v, speed_frame, position_frame in islice(iter_setpoints(plan, node), self.PREVIEW_ROWS):
            self.frames.insert('', 'end', values=(f'{t:.3f}', f'{p:.2f}', v, speed_frame, position_frame))
        tail = f' · 预览前 {self.PREVIEW_ROWS} 点' if count > self.PREVIEW_ROWS else ''
        self.plan_text.set(f'ID{node} · {plan.start_deg:.2f}° → {plan.target_deg:.2f}° · '
                           f'{count} 点 / {count * 2} 个位置与速度帧 · '
                           f'{plan.duration:.3f} s · 终点速度 0 rpm{tail}')
        if update_plot:
            self.plot_trial = None
            self.plot_page.show_plan(preview_trajectory(plan.start_deg, payload))
        return plan, node, payload

    def export_frames(self):
        result = self.generate(update_plot=False)
        if result is None:
            return
        plan, node, _ = result
        path = filedialog.asksaveasfilename(parent=self.root, defaultextension='.csv',
                initialfile=f'MIT_ID{node}_trajectory.csv', filetypes=[('CSV', '*.csv')])
        if path:
            try:
                with open(path, 'w', newline='', encoding='utf-8-sig') as file:
                    rows = csv.writer(file)
                    rows.writerow(('时间_s', '输出角度_deg', '电机转速_rpm', '速度帧_HEX', '位置帧_HEX'))
                    rows.writerows(iter_setpoints(plan, node))
                self.note.set(f'已导出 {path}')
            except OSError as error:
                self.note.set(f'导出失败：{error}')

    def start(self):
        result = self.generate(update_plot=False)
        if result is None:
            return
        plan, node, payload = result
        confirmation = (f'ID{node} · 目标 {plan.target_deg:.2f}° · 电机端参考上限 {plan.max_motor_rpm} rpm\n'
                        f'从启动时读取的位置开始，参考到终点后保持 {plan.hold_seconds:g} 秒并停机。\n'
                        '确认机构允许本次行程，且现场可以立即停止或断电。')
        if messagebox.askokcancel('发送单次 MIT 轨迹', confirmation, parent=self.root):
            self.motion = 'starting'
            self.stop_monitor('开始 MIT 轨迹，已停止 M0 采样。轨迹实测反馈可在“速度与位置”页查看。')
            self.monitor_pending = None
            self.refresh_controls()
            self.submit('start_mit_trajectory', payload)

    def display_reply(self, summary, rows):
        self.reply_summary.set(summary)
        self.reply_fields.delete(*self.reply_fields.get_children())
        for row in rows:
            self.reply_fields.insert('', 'end', values=row)

    def parse_input(self):
        try:
            self.display_reply(*parse_exchange(self.tx.get(), self.rx.get()))
        except ValueError as error:
            self.display_reply(str(error), ())

    def note_wire(self, data):
        tx, rx = data['tx'], data['rx']
        try:
            summary, _ = parse_exchange(tx, rx)
        except ValueError as error:
            summary = str(error)
        error = data.get('error')
        if error:
            summary = error
        item = self.wire.insert('', 'end', values=(tx, rx, summary))
        self.wire_ids.append(item)
        self.wire_data[item] = tx, rx
        if len(self.wire_ids) > self.WIRE_ROWS:
            old = self.wire_ids.popleft()
            self.wire.delete(old)
            del self.wire_data[old]
        if not self.wire.selection():
            self.wire.see(item)

    def select_wire(self, _event):
        selected = self.wire.selection()
        if selected and selected[0] in self.wire_data:
            tx, rx = self.wire_data[selected[0]]
            self.tx.set(tx)
            self.rx.set(rx)
            self.parse_input()

    def response(self, event, data):
        if event == 'wire':
            self.note_wire(data)
        elif event == 'port_opened' and data['baudrate'] == 1000000:
            self.link = 'loader'
            self.status.set(f'{data["port"]} · 1 Mbps 升级串口已打开')
            self.refresh_controls()
        elif event.startswith('firmware_'):
            self.flash_page.result(event, data)
            if event == 'firmware_state':
                self.flash_busy = data['busy']
                self.refresh_controls()
        elif event == 'plot_plan':
            self.plot_trial = data['id']
            self.plot_page.show_plan(data['preview'])
            self.tabs.select(self.plot_page)
        elif event == 'plot_feedback':
            if data['id'] == self.plot_trial:
                self.plot_page.add_feedback(data['elapsed'], data['position_deg'], data['speed_rpm'])
        elif event == 'status':
            self.latest_state = data
            marker = self.session.identity['marker']
            self.firmware_marker = marker
            if self.link == 'opening':
                self.link = 'ready'
                self.mit_supported = supports_mit(marker)
                self.note.set(f'已读回 0x{marker:04X}' if self.mit_supported else '该固件不支持 MIT 轨迹。')
                self.refresh_controls()
            self.status.set(f'ID{self.node.get()} · 0x{marker:04X} · 模式 {data["mode"]} · '
                            f'位置 {data["position_actual_deg"]:.2f}° · 转速 {data["speed_actual_rpm"]} rpm · '
                            f'目标/实测电流 {data["current_target_ma"]}/{data["current_feedback_magnitude_ma"]} mA · '
                            f'状态 0x{data["motor_status"]:02X} · 故障 {data["fault"]}')
        elif event == 'register_words':
            self.register_pages.set_words(data['address'], data['values'])
        elif event == 'monitor_sample':
            self.monitor_result(data)
        elif event == 'register_done':
            self.register_busy = False
            self.refresh_controls()
        elif event == 'manual_state':
            self.manual_running = data
            self.refresh_controls()
        elif event == 'node_changed':
            self.node.set(str(data))
            self.latest_state = None
            self.stop_monitor('站号已切换，已停止采样。')
            self.register_pages.clear()
        elif event == 'mit_trajectory_progress':
            phase = {'accelerating': '加速', 'cruising': '匀速', 'decelerating': '减速',
                     'holding': '终点保持'}[data['phase']]
            self.motion_text.set(f'{phase} · 已发 {data["points_sent"]} 点 · '
                                 f'参考 {data["position_des_deg"]:.2f}° / {data["velocity_des_rpm"]:.1f} rpm')
        elif event == 'mit_trajectory_state':
            self.motion = 'active' if data['active'] else 'idle'
            self.note.set('正在发送轨迹…' if data['active'] else
                          f'{data["reason"]} · ' + ('已确认停机' if data['stop_confirmed'] else '停机待确认'))
            self.refresh_controls()
        elif event == 'mit_trajectory_error':
            self.note.set(data['error'] + (' · 已确认停机' if data['stop_confirmed'] else ' · 停机待确认'))
        elif event == 'disconnected':
            self.link, self.motion = 'closed', 'idle'
            self.latest_state = None
            self.mit_supported = False
            self.firmware_marker = None
            self.register_busy = self.manual_running = False
            self.stop_monitor('串口已断开，采样已停止。')
            self.monitor_pending = None
            self.register_pages.clear()
            self.status.set('串口已断开')
            self.serial_format.set('3 Mbps · 8N1')
            self.refresh_controls()
        elif event == 'note':
            if data.startswith('停止指令及停机状态已确认') and self.motion == 'stopping':
                self.motion = 'idle'
                self.register_busy = False
                self.monitor_pending = None
                self.schedule_monitor()
                self.refresh_controls()
            if not data.startswith(('串口已打开，发送0字节', '已断开；')):
                self.note.set(data)

    def pump(self):
        for _ in range(160):
            try:
                event, data = self.events.get_nowait()
            except queue.Empty:
                break
            self.response(event, data)
        self.root.after(30, self.pump)

    def close(self):
        if self.flash_busy:
            self.tabs.select(self.flash_page)
            self.note.set('烧录页操作进行中。请等待结果或在本页请求中止。')
            return
        if self.session is None:
            self.root.destroy()
            return
        if not self.closing:
            self.closing = True
            self.stop_monitor('正在退出，采样已停止。')
            self.note.set('正在结束串口会话…')
            self.refresh_controls()
            self.session.shutdown.set()
        if self.thread.is_alive():
            self.root.after(100, self.close)
        else:
            if self.session.close_error:
                messagebox.showerror('退出时停机待确认', self.session.close_error, parent=self.root)
            self.root.destroy()


def main():
    root = tk.Tk()
    TrajectoryHost(root)
    root.mainloop()


if __name__ == '__main__':
    main()
