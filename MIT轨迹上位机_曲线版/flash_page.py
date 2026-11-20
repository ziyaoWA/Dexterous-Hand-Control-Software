"""485 firmware operations submitted to the trajectory host's serial worker."""

from datetime import datetime
import json
from pathlib import Path
import tkinter as tk
from tkinter import filedialog, messagebox, ttk


_OPERATIONS = {
    'validate': '本地校验 HEX',
    'enter': '进入升级态',
    'read_info': '读取升级状态',
    'dry_run': '演练传输',
    'upgrade': '烧录固件',
}
_FRAMES = {'rtu_tx': '3 Mbps 发送', 'rtu_rx': '3 Mbps 接收',
           'tx': '1 Mbps 发送', 'rx': '1 Mbps 接收'}
_SINGLE_BOARD = '请确认总线仅连接当前一块驱动板。'


class FlashPage(ttk.Frame):
    LOG_LINES = 240

    def __init__(self, parent, submit_callback, open_loader_callback, cancel_callback):
        super().__init__(parent, padding=10)
        self.submit_callback = submit_callback
        self.cancel_callback = cancel_callback
        self.link, self.port, self.node = 'closed', '', ''
        self.available = self.busy = False
        self.image = None
        self.operation = None
        self.cancel_requested = False
        self.writing = False
        self.path = tk.StringVar()
        self.file_details = tk.StringVar(value='请选择项目的应用程序 HEX。')
        self.link_details = tk.StringVar()
        self.status = tk.StringVar(value='等待选择固件')
        self.progress = tk.DoubleVar()
        self._build_file()
        self._build_actions(open_loader_callback)
        self._build_log()
        self.path.trace_add('write', self._invalidate)
        self._refresh_buttons()

    def _build_file(self):
        panel = ttk.LabelFrame(self, text='应用固件', padding=8)
        panel.pack(fill='x')
        row = ttk.Frame(panel)
        row.pack(fill='x')
        self.path_entry = ttk.Entry(row, textvariable=self.path)
        self.path_entry.pack(side='left', fill='x', expand=True)
        self.choose_button = ttk.Button(row, text='选择 HEX…', command=self._choose)
        self.choose_button.pack(side='left', padx=6)
        self.validate_button = ttk.Button(row, text='校验 HEX', command=self._validate)
        self.validate_button.pack(side='left')
        ttk.Label(panel, textvariable=self.file_details, wraplength=780).pack(anchor='w', pady=(5, 0))

    def _build_actions(self, open_loader):
        panel = ttk.LabelFrame(self, text='串口与烧录', padding=8)
        panel.pack(fill='x', pady=(8, 0))
        ttk.Label(panel, textvariable=self.link_details).pack(anchor='w')
        ttk.Label(panel, text='正常固件使用顶部 3 Mbps 控制连接。已处于升级态时打开 1 Mbps 串口。升级帧无站号，总线请只接目标板。',
                  wraplength=780).pack(anchor='w', pady=(3, 6))
        actions = ttk.Frame(panel)
        actions.pack(fill='x')
        self.loader_button = self._button(actions, 0, '打开 1 Mbps 串口', open_loader)
        self.enter_button = self._button(actions, 1, '进入升级态', lambda: self._device_action('enter'))
        self.info_button = self._button(actions, 2, '读升级状态', lambda: self._device_action('info'))
        self.dry_button = self._button(actions, 3, '传输演练', lambda: self._device_action('dry_run'))
        self.upgrade_button = self._button(actions, 4, '开始烧录…', lambda: self._device_action('upgrade'))
        self.cancel_button = self._button(actions, 5, '中止本轮', self._cancel)
        self.gauge = ttk.Progressbar(panel, variable=self.progress, maximum=256, mode='determinate')
        self.gauge.pack(fill='x', pady=(8, 3))
        ttk.Label(panel, textvariable=self.status, wraplength=780).pack(anchor='w')

    @staticmethod
    def _button(panel, column, label, command):
        button = ttk.Button(panel, text=label, command=command)
        button.grid(row=0, column=column, sticky='ew', padx=(0 if column == 0 else 4, 0))
        panel.columnconfigure(column, weight=1)
        return button

    def _build_log(self):
        panel = ttk.Frame(self)
        panel.pack(fill='both', expand=True, pady=(8, 0))
        ttk.Label(panel, text='升级回包与诊断').grid(row=0, column=0, sticky='w')
        ttk.Button(panel, text='清屏', command=self._clear_log).grid(row=0, column=0, sticky='e')
        self.diagnostics = tk.Text(panel, height=6, wrap='none', state='disabled', font=('Consolas', 9))
        self.diagnostics.grid(row=1, column=0, sticky='nsew', pady=(4, 0))
        vertical = ttk.Scrollbar(panel, orient='vertical', command=self.diagnostics.yview)
        horizontal = ttk.Scrollbar(panel, orient='horizontal', command=self.diagnostics.xview)
        vertical.grid(row=1, column=1, sticky='ns', pady=(4, 0))
        horizontal.grid(row=2, column=0, sticky='ew')
        self.diagnostics.configure(yscrollcommand=vertical.set, xscrollcommand=horizontal.set)
        panel.rowconfigure(1, weight=1)
        panel.columnconfigure(0, weight=1)

    def set_state(self, link, available, busy, port, node):
        self.link, self.available, self.busy = link, available, busy
        self.port, self.node = port, node
        self.link_details.set({
            'ready': f'{port} · ID{node} · 3 Mbps 控制连接',
            'loader': f'{port} · 1 Mbps 升级连接',
            'opening': f'正在打开 {port}',
            'disconnecting': '正在关闭串口',
            'closed': '串口未连接',
        }[link])
        self._refresh_buttons()

    def _refresh_buttons(self):
        free = self.available and not self.busy
        connected = self.link in ('ready', 'loader')
        editable = free and (connected or self.link == 'closed')
        enabled = (
            (self.path_entry, editable),
            (self.choose_button, editable),
            (self.validate_button, editable and bool(self.path.get().strip())),
            (self.loader_button, free and self.link == 'closed'),
            (self.enter_button, free and self.link == 'ready'),
            (self.info_button, free and self.link == 'loader'),
            (self.dry_button, free and connected and self.image is not None),
            (self.upgrade_button, free and connected and self.image is not None),
            (self.cancel_button, self.busy and self.operation not in (None, 'validate')
             and not self.cancel_requested),
        )
        for widget, active in enabled:
            widget.configure(state='normal' if active else 'disabled')

    def _choose(self):
        path = filedialog.askopenfilename(parent=self, title='选择应用固件', filetypes=[('Intel HEX', '*.hex')])
        if path:
            self.path.set(path)

    def _invalidate(self, *_):
        self.image = None
        self.file_details.set('请校验所选 HEX。' if self.path.get().strip() else '请选择项目的应用程序 HEX。')
        self._refresh_buttons()

    @staticmethod
    def _is_legacy(image):
        return image.get('requires_power_cycle') is True or image.get('normal_marker') == 0x1111

    def _start(self, operation, payload):
        self.operation = 'read_info' if operation == 'info' else operation
        self.cancel_requested = self.writing = False
        self.progress.set(0)
        self.gauge.configure(maximum=256)
        self.submit_callback(f'firmware_{operation}', payload)

    def _validate(self):
        path = self.path.get().strip()
        if self.busy or not self.available or not path:
            return
        self.image = None
        self.file_details.set('正在检查文件格式与固件标识…')
        self._start('validate', {'path': path})

    def _device_action(self, operation):
        if self.busy or not self.available or self.link not in ('ready', 'loader'):
            return
        if (operation == 'enter' and self.link != 'ready') or (operation == 'info' and self.link != 'loader'):
            return
        payload = {}
        if operation in ('dry_run', 'upgrade'):
            if self.image is None:
                return
            payload = {'path': self.image['path'], 'expected_sha256': self.image['sha256']}
            if operation == 'upgrade':
                if not self._confirm_write():
                    return
                payload['allow_legacy'] = self._is_legacy(self.image)
            elif not messagebox.askyesno('传输演练', _SINGLE_BOARD +
                    '演练会进入升级态并传输应用数据，Flash 保持原样。', parent=self):
                return
        elif operation == 'enter' and not messagebox.askyesno('进入升级态', _SINGLE_BOARD +
                f'程序将停止 ID{self.node} 并发送 0x42，随后设备以 1 Mbps 进入升级态。', parent=self):
            return
        self._start(operation, payload)

    def _confirm_write(self):
        image = self.image
        version = f"0x{image['normal_marker']:04X}"
        target = f'{self.port} · ID{self.node}' if self.link == 'ready' else f'{self.port} · 1 Mbps 升级连接'
        if not messagebox.askyesno('确认烧录',
                f"连接 {target}\n固件 {Path(image['path']).name} · {version}\n"
                f"SHA256 {image['sha256']}\n\n"
                + _SINGLE_BOARD + '演练通过后将擦写 32 KiB 应用区，保留 EEPROM。'
                '烧录期间请保持供电。是否开始？', parent=self):
            return False
        return (not self._is_legacy(image) or messagebox.askyesno('确认烧录 0x1111 旧版',
                '旧版写入校验通过后，设备会留在升级态。届时请断电重上电，再通过顶部 3 Mbps 控制连接读回版本。'
                '是否烧录旧版？', parent=self))

    def _cancel(self):
        if self.busy and not self.cancel_requested and self.operation not in (None, 'validate'):
            self.cancel_requested = True
            self.cancel_callback()
            self.status.set('已请求中止，正在等待设备应答。请保持供电。')
            self._refresh_buttons()

    def result(self, event, data):
        if event == 'firmware_state':
            self.busy = data['busy']
            self.operation = data['operation'] if self.busy else None
            if self.busy and not self.cancel_requested:
                self.status.set(_OPERATIONS.get(self.operation, self.operation) + '进行中…')
            self._refresh_buttons()
        elif event == 'firmware_event':
            self._on_progress(data['kind'], data['data'])
        elif event == 'firmware_error':
            if data['operation'] == 'validate':
                self.file_details.set('HEX 校验未通过。')
            detail = '已进入实际擦写阶段，请保持供电并查看日志。' if self.writing else ''
            self.status.set(f"操作未完成。{data['error']} {detail}".strip())
            self._append_log('错误', data)
        elif event == 'firmware_done':
            self._on_completion(data['operation'], data['result'])
            self._append_log('结果', data)

    def _on_progress(self, kind, value):
        if kind in ('kernel_progress', 'progress'):
            if kind == 'kernel_progress':
                text, done = '加载 RAM 内核', value['blocks']
            else:
                text = '写入应用区' if value['stage'] == 'physical' else '传输演练'
                done = value['pages']
            self.gauge.configure(maximum=value['total'])
            self.progress.set(done)
            if not self.cancel_requested:
                self.status.set(f"{text} {done}/{value['total']}")
            return
        if kind == 'stage' and value == '实际擦写应用区':
            self.writing = True
        if kind in ('stage', 'preparing_checks', 'physical_verified', 'reset_acknowledgement') and not self.cancel_requested:
            self.status.set({'physical_verified': '应用区和 EEPROM 校验通过，正在确认启动。',
                             'reset_acknowledgement': '已收到复位应答，正在读取应用版本。'}.get(kind, str(value)))
        self._append_log(_FRAMES.get(kind, kind), value)

    def _on_completion(self, operation, data):
        if not isinstance(data, dict):
            self.status.set('操作结果缺少核验信息，请查看日志。')
            return
        if operation == 'validate':
            required = ('path', 'sha256', 'size', 'image_crc32', 'normal_marker')
            if not all(key in data for key in required):
                self.file_details.set('校验结果缺少固件信息，请重新校验。')
                self.status.set('固件信息不完整。')
                return
            try:
                same_file = Path(self.path.get().strip()).resolve() == Path(data['path'])
            except (OSError, ValueError):
                same_file = False
            if not same_file:
                self.file_details.set('所选文件已更改，请重新校验。')
                self.status.set('等待校验当前文件。')
                return
            self.image = data
            legacy = ' · 0x1111 需断电重上电后核验' if self._is_legacy(data) else ''
            self.file_details.set(
                f"目标版本 0x{data['normal_marker']:04X} · {data['size']} 字节 · "
                f"CRC32 0x{data['image_crc32']:08X}{legacy}\nSHA256 {data['sha256']}")
            self.status.set('本地 HEX 校验通过。')
        elif operation == 'upgrade':
            if self._is_legacy(data.get('image', {})) or data.get('requires_power_cycle') is True:
                self.status.set('旧版应用区及 EEPROM 校验通过。请断电重上电，再用顶部 3 Mbps 连接读回应用版本。'
                                if data.get('physical_write_verified') is True else '缺少旧版写入校验结果，请查看日志。')
            else:
                self.status.set('烧录成功。应用区 CRC 和 EEPROM 校验通过，复位后版本读回一致。'
                                if data.get('verified') is True else '缺少复位后版本核验结果，请查看日志。')
        elif operation == 'dry_run':
            self.status.set('传输演练通过。Flash 保持原样，设备仍在升级态。继续时重新打开 1 Mbps 串口。'
                            if data.get('dry_run_verified') is True and data.get('physical_write') is False
                            else '缺少传输演练校验结果，请查看日志。')
        else:
            marker, state = data.get('marker'), data.get('state')
            stage = f' · 升级器 0x{marker:04X} · 状态 {state}' if isinstance(marker, int) else ''
            self.status.set(f'设备处于升级态{stage}。继续时重新打开 1 Mbps 串口。')

    def _append_log(self, label, value):
        detail = value if isinstance(value, str) else json.dumps(value, ensure_ascii=False, default=str)
        time_text = datetime.now().strftime('%H:%M:%S')
        self.diagnostics.configure(state='normal')
        self.diagnostics.insert('end', f'{time_text}  {label}  {detail}\n')
        end_line = int(self.diagnostics.index('end-1c').split('.')[0])
        if end_line > self.LOG_LINES:
            self.diagnostics.delete('1.0', f'{end_line - self.LOG_LINES + 1}.0')
        self.diagnostics.see('end')
        self.diagnostics.configure(state='disabled')

    def _clear_log(self):
        self.diagnostics.configure(state='normal')
        self.diagnostics.delete('1.0', 'end')
        self.diagnostics.configure(state='disabled')
