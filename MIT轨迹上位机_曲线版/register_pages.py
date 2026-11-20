"""Explicit register pages sharing the trajectory window's serial worker."""

from dataclasses import dataclass
from decimal import Decimal, InvalidOperation
import tkinter as tk
from tkinter import messagebox, ttk

from protocol_services import GROUPS
from trajectory_wire import _engineering


@dataclass(frozen=True)
class _Unit:
    label: str
    scale: int = 1
    signed: bool = False
    note: str = ''


_RAW = _Unit('原始整数')
_UNITS = {
    1: _Unit('rpm', signed=True),
    2: _Unit('°', 100, True),
    3: _Unit('A', 1000, True),
    4: _Unit('Q15原始整数'),
    5: _Unit('Q15原始整数'),
    6: _Unit('Q12原始整数'),
    7: _Unit('Q12原始整数'),
    8: _Unit('Q15原始整数'),
    9: _Unit('Q12原始整数'),
    10: _Unit('Q12原始整数'),
    12: _Unit('rpm', note='0采用固件默认速度上限。'),
    13: _Unit('A', 1000, note='0禁止驱动。'),
    14: _Unit('rpm/s/mA', 10, note='模型增益b0；原始值为b0×10，0采用固件默认。'),
    15: _Unit('rad/s', note='0采用固件默认；实际生效范围由固件处理。'),
    16: _Unit('Q10原始整数', note='1024为全量补偿，0只观测不补偿。'),
    17: _Unit('A', 1000, True, '只读ESO补偿电流。'),
    18: _Unit('Q25原始整数', note='0关闭位置积分。'),
    19: _Unit('Q12原始整数', note='0关闭位置阻尼。'),
    0x100: _Unit('A', 1000, note='额定电流；0采用固件默认。'),
    0x101: _Unit('ms', note='0关闭这一级过流保护及其降额。'),
    0x102: _Unit('ms', note='0关闭这一级过流保护及其降额。'),
    0x103: _Unit('ms', note='0关闭这一级过流保护及其降额。'),
    0x104: _Unit('ms', note='0关闭通信超时。'),
    0x105: _Unit('counts', note='0采用固件默认位置跳变阈值。'),
    0x106: _Unit('rpm', note='0采用固件默认堵转速度阈值。'),
    0x107: _Unit('%额定电流', note='阈值=额定电流×原始值/100；150表示1.5倍额定，0采用默认。'),
    0x108: _Unit('V', 100, note='1个原始单位为10mV；0采用NVM默认。'),
    0x109: _Unit('V', 100, note='1个原始单位为10mV；0采用NVM默认。'),
    0x10A: _Unit('℃', note='0采用固件默认过温阈值。'),
    0x10B: _Unit('%额定电流', note='当前固件按额定电流×原始值/100计算；0采用默认。'),
    0x10C: _Unit('ms', note='0采用固件默认故障确认时间。'),
    0x10D: _Unit('°', 100, note='仅0x1114支持；0采用默认0.05°。'),
    0x10E: _Unit('A', 1000, note='仅0x1114支持；0跟随F0-13运行限流。'),
    0x10F: _Unit('°', 100, note='仅0x1114支持；0采用默认额外0.05°滞回。'),
    0x6002: _Unit('rpm', signed=True),
    0x6003: _Unit('°', 100, True),
    0x6004: _Unit('rpm', signed=True),
    0x6005: _Unit('A', 1000, True),
    0x600B: _Unit('V', 100),
    0x600C: _Unit('A', 1000, note='峰值幅值，不代表带符号转矩电流。'),
    0x600D: _Unit('rpm', signed=True),
    0x600E: _Unit('°', 100, True, '16位饱和位置反馈；连续位置见M0-27/28。'),
    0x6011: _Unit('°', 100, True),
    0x6016: _Unit('ms'),
    0x6017: _Unit('℃', signed=True),
    0x6027: _Unit('rpm', signed=True),
    0x6028: _Unit('rpm', signed=True),
    0x6029: _Unit('rpm', signed=True),
    0x602A: _Unit('rpm', signed=True),
    0x602C: _Unit('A', 1000, True),
    0x602D: _Unit('A', 1000, note='命令域电流上限，尚未乘过流降额比例。'),
}

_LABELS = {
    0x107: '堵转电流阈值（相对额定电流）',
    0x10B: '过压电流上限（相对额定电流）',
}
_PAGE_GROUPS = (('F0', 'F0', 20), ('F1', 'F1', 16), ('M0', 'M0', 50), ('C0', 'H', 2))
_MONITOR_PAIRS = (0x601B, 0x601D, 0x601F, 0x6025)


def _decimal_text(number):
    text = format(number, 'f')
    if '.' in text:
        text = text.rstrip('0').rstrip('.')
    return '0' if text in ('', '-0') else text


def parse_register_value(address, text, raw=False):
    """Convert exactly to a 16-bit wire word; firmware applies register policy."""
    try:
        number = Decimal(str(text).strip())
    except InvalidOperation as error:
        raise ValueError('请输入有效数值。') from error
    if not number.is_finite():
        raise ValueError('数值必须是有限数。')
    unit = _UNITS.get(address, _RAW)
    # Every engineering scale here is a power of ten. Moving the exponent
    # preserves every input digit, including digits beyond Decimal's context.
    sign, digits, exponent = number.as_tuple()
    scaled = number if raw else Decimal((sign, digits, exponent + len(str(unit.scale)) - 1))
    if scaled != scaled.to_integral_value():
        detail = '原始字必须为整数。' if raw else f'该项的最小可写步长为{_decimal_text(Decimal(1) / unit.scale)} {unit.label}。'
        raise ValueError(detail)
    low = -32768 if raw or unit.signed else 0
    high = 65535 if raw or not unit.signed else 32767
    if not low <= scaled <= high:
        raise ValueError('换算结果超出16位寄存器表示范围。')
    return int(scaled) & 0xFFFF


def display_register_value(address, word):
    word = int(word) & 0xFFFF
    if address in _UNITS:
        unit = _UNITS[address]
        value = word - 65536 if unit.signed and word >= 32768 else word
        result = f'{_decimal_text(Decimal(value) / unit.scale)} {unit.label}'
        if address in (0x107, 0x10B):
            result += f'（{_decimal_text(Decimal(word) / 100)}×额定）' if word else '（采用默认）'
        return result
    return _engineering(address, word)


class RegisterPages:
    """Four ttk tabs; every serial operation goes through supplied callbacks."""

    def __init__(self, notebook, request_callback, stop_callback, status_callback):
        self.notebook = notebook
        self.request_callback = request_callback
        self.stop_callback = stop_callback
        self.status_callback = status_callback
        self.connected = self.busy = self.trajectory_active = False
        self.marker = None
        self.words = {}
        self.tables, self.frames, self.editors = {}, {}, {}
        self.read_buttons, self.action_buttons = [], []
        self.stop_button = None
        self.message = tk.StringVar(value='读取后显示当前值；编辑设定值不会自动发送。')
        for page, group, count in _PAGE_GROUPS:
            self._make_page(page, group, count)
        self.set_state(False, False, False, None)

    def _make_page(self, page, group, count):
        frame = self.frames[page] = ttk.Frame(self.notebook, padding=10)
        label = {'F0': '控制参数', 'F1': '保护参数', 'M0': '监控参数', 'C0': '通讯命令'}[page]
        self.notebook.add(frame, text=f'{page} {label}')
        if page == 'M0':
            self.monitor_tabs = ttk.Notebook(frame)
            self.monitor_tabs.pack(fill='both', expand=True)
            frame = ttk.Frame(self.monitor_tabs, padding=6)
            self.monitor_tabs.add(frame, text='寄存器')
        base, names = GROUPS[group]
        toolbar = ttk.Frame(frame)
        toolbar.pack(fill='x', pady=(0, 8))
        title = {'F0': '控制与环路参数', 'F1': '保护与位置保持参数', 'M0': '运行监控 · 只读', 'C0': '控制命令 · 地址0x7000'}[page]
        ttk.Label(toolbar, text=title).pack(side='left', padx=(0, 14))
        for title, callback in (
                ('读取本页', lambda b=base, n=count: self._read(b, n)),
                ('读取选中', lambda p=page: self._read_selected(p))):
            button = ttk.Button(toolbar, text=title, command=callback)
            button.pack(side='left', padx=3)
            self.read_buttons.append(button)

        area = ttk.Frame(frame)
        area.pack(fill='both', expand=True)
        columns = ('code', 'address', 'name', 'engineering', 'raw', 'hex')
        tree = self.tables[page] = ttk.Treeview(area, columns=columns, show='headings', height=12)
        for key, title, width in (
                ('code', '功能码', 76), ('address', '地址', 86), ('name', '参数', 270),
                ('engineering', '当前工程量', 230), ('raw', '原始字', 90), ('hex', 'HEX', 84)):
            tree.heading(key, text=title)
            tree.column(key, width=width, minwidth=60, stretch=key in ('name', 'engineering'))
        scroll = ttk.Scrollbar(area, orient='vertical', command=tree.yview)
        horizontal = ttk.Scrollbar(area, orient='horizontal', command=tree.xview)
        tree.configure(yscrollcommand=scroll.set, xscrollcommand=horizontal.set)
        tree.grid(row=0, column=0, sticky='nsew')
        scroll.grid(row=0, column=1, sticky='ns')
        horizontal.grid(row=1, column=0, sticky='ew')
        area.rowconfigure(0, weight=1)
        area.columnconfigure(0, weight=1)
        for index in range(count):
            address = base + index
            name = _LABELS.get(address, names[index])
            tree.insert('', 'end', iid=str(address), values=(f'{page}-{index:02d}', f'0x{address:04X}', name, '—', '—', '—'))
        if page in ('F0', 'F1'):
            self._make_editor(frame, page)
            tree.bind('<<TreeviewSelect>>', lambda unused, p=page: self._select(p))
            tree.bind('<Double-1>', lambda unused, p=page: self._focus_editor(p))
        elif page == 'C0':
            self._make_commands(frame)
        else:
            ttk.Label(frame, text='M0全组只读。原始高低字分别显示；累计计数与位置误差按有符号32位解释。', wraplength=1000).pack(anchor='w', pady=(8, 0))
        ttk.Label(frame, textvariable=self.message, wraplength=1000).pack(anchor='w', pady=(8, 0))

    def _make_editor(self, frame, page):
        panel = ttk.LabelFrame(frame, text='选中项设定', padding=8)
        panel.pack(fill='x', pady=(8, 0))
        title = tk.StringVar(value='选择一项参数')
        note = tk.StringVar(value='')
        unit = tk.StringVar(value='')
        value = tk.StringVar()
        mode = tk.StringVar(value='工程量')
        ttk.Label(panel, textvariable=title).grid(row=0, column=0, columnspan=5, sticky='w')
        kind = ttk.Combobox(panel, textvariable=mode, values=('工程量', '原始字'), state='readonly', width=9)
        kind.grid(row=1, column=0, padx=(0, 8), pady=6)
        entry = ttk.Entry(panel, textvariable=value, width=22)
        entry.grid(row=1, column=1, padx=(0, 8), pady=6)
        ttk.Label(panel, textvariable=unit).grid(row=1, column=2, sticky='w', padx=(0, 12))
        button = ttk.Button(panel, text='写入并读回', command=lambda p=page: self._write(p))
        button.grid(row=1, column=3, padx=4)
        ttk.Label(panel, textvariable=note, wraplength=1000).grid(row=2, column=0, columnspan=5, sticky='w')
        panel.columnconfigure(4, weight=1)
        self.editors[page] = dict(title=title, note=note, unit=unit, value=value, mode=mode,
                                  kind=kind, entry=entry, button=button)
        kind.bind('<<ComboboxSelected>>', lambda unused, p=page: self._select(p))

    def _make_commands(self, frame):
        commands = ttk.LabelFrame(frame, text='执行控制命令', padding=8)
        commands.pack(fill='x', pady=(8, 0))
        for index, (title, command) in enumerate((
                ('命令清零', lambda: self._command(0x7000, 0)),
                ('RUN', lambda: self._command(0x7000, 1)),
                ('清故障', lambda: self._command(0x7000, 5)),
                ('STOP', self._stop),
                ('保存F0/F1', lambda: self._command(0x7001, 1)),
                ('恢复默认并保存', lambda: self._command(0x7001, 2)),
                ('重上电后核验保存', self._verify_save))):
            button = ttk.Button(commands, text=title, command=command)
            button.grid(row=index // 4, column=index % 4, padx=4, pady=4, sticky='ew')
            if title == 'STOP':
                self.stop_button = button
            else:
                self.action_buttons.append(button)
        for column in range(4):
            commands.columnconfigure(column, weight=1)
        ttk.Label(commands, text='点击 STOP 停机。保存后可重上电核验。恢复默认会覆盖参数并将站号改回 ID1。',
                  wraplength=1000).grid(row=2, column=0, columnspan=4, sticky='w', padx=4, pady=(4, 0))

    def _ready(self):
        return self.connected and not self.busy and not self.trajectory_active

    def _can_write(self, address):
        return (self._ready() and self.marker is not None and address != 17
                and not (0x10D <= address <= 0x10F and self.marker != 0x1114))

    def _notice(self, text):
        self.message.set(text)
        self.status_callback(text)

    def _dispatch(self, payload):
        if not self._ready():
            self._notice('连接空闲后可操作寄存器；轨迹运行期间请先STOP。')
            return
        try:
            self.request_callback(payload)
        except Exception as error:
            self._notice(str(error))
            messagebox.showerror('操作未发送', str(error), parent=self.notebook.winfo_toplevel())

    def _read(self, address, count):
        self._dispatch({'action': 'read', 'address': address, 'count': count})

    def _read_selected(self, page):
        selected = self.tables[page].selection()
        if not selected:
            self._notice('先选择一行。')
            return
        address = int(selected[0])
        for high in _MONITOR_PAIRS:
            if address in (high, high + 1):
                self._read(high, 2)
                return
        self._read(address, 1)

    def _select(self, page):
        selected = self.tables[page].selection()
        editor = self.editors[page]
        if not selected:
            editor['button'].configure(state='disabled')
            editor['entry'].configure(state='disabled')
            return
        address = int(selected[0])
        row = self.tables[page].item(selected[0], 'values')
        unit = _UNITS.get(address, _RAW)
        raw = editor['mode'].get() == '原始字'
        editor['title'].set(f'{row[0]} · {row[2]}')
        editor['unit'].set('16位原始整数（DEC）' if raw else unit.label)
        note = unit.note
        if address == 17:
            note = 'F0-17为只读ESO输出。'
        elif 0x10D <= address <= 0x10F and self.marker != 0x1114:
            note = '当前固件未识别为0x1114，此项为保留字，不能写入。'
        editor['note'].set(note)
        word = self.words.get(address)
        if word is None:
            editor['value'].set('')
        else:
            value = word - 65536 if not raw and unit.signed and word >= 32768 else word
            editor['value'].set(str(word) if raw else _decimal_text(Decimal(value) / unit.scale))
        enabled = self._can_write(address)
        editor['entry'].configure(state='normal' if enabled else 'disabled')
        editor['button'].configure(state='normal' if enabled else 'disabled')

    def _focus_editor(self, page):
        self._select(page)
        entry = self.editors[page]['entry']
        if str(entry.cget('state')) != 'disabled':
            entry.focus_set()
            entry.selection_range(0, 'end')

    def _write(self, page):
        selected = self.tables[page].selection()
        if not selected:
            return self._notice('先选择一行。')
        address = int(selected[0])
        if not self._can_write(address):
            return self._notice('当前不能写入该寄存器。')
        editor = self.editors[page]
        try:
            word = parse_register_value(address, editor['value'].get(), editor['mode'].get() == '原始字')
        except ValueError as error:
            self._notice(str(error))
            return messagebox.showerror('数值未接受', str(error), parent=self.notebook.winfo_toplevel())
        self._dispatch({'action': 'write', 'address': address, 'value': word})

    def _command(self, address, value):
        if not self._ready() or self.marker is None:
            return self._notice('先连接并识别固件，待通信空闲后执行命令。')
        if address == 0x7000 and value == 1:
            if not messagebox.askokcancel('执行RUN', '按设备当前F0模式和给定值启动运行。确认执行RUN？', parent=self.notebook.winfo_toplevel()):
                return
        if address == 0x7001 and value == 2:
            if not messagebox.askokcancel('恢复默认并保存',
                    '此操作将覆盖 F0/F1 参数并将设备站号恢复为 ID1。请确认总线上没有另一台 ID1。',
                    parent=self.notebook.winfo_toplevel()):
                return
        self._dispatch({'action': 'write', 'address': address, 'value': value})

    def _stop(self):
        if self.connected:
            self.stop_callback()

    def _verify_save(self):
        if not self._ready() or self.marker is None:
            return
        if messagebox.askokcancel('重上电后核验保存', '仅在已完成断电重上电后继续。现在读取F0/F1，核对上次保存请求的参数？', parent=self.notebook.winfo_toplevel()):
            self._dispatch({'action': 'verify_save'})

    def set_state(self, connected, busy, trajectory_active, marker):
        marker_changed = self.marker != marker
        self.connected = bool(connected)
        self.busy = bool(busy)
        self.trajectory_active = bool(trajectory_active)
        self.marker = marker
        enabled = 'normal' if self._ready() else 'disabled'
        for button in self.read_buttons:
            button.configure(state=enabled)
        for button in self.action_buttons:
            button.configure(state=enabled if marker is not None else 'disabled')
        self.stop_button.configure(state='normal' if self.connected else 'disabled')
        for page, editor in self.editors.items():
            editor['kind'].configure(state='readonly' if self._ready() else 'disabled')
            selected = self.tables[page].selection()
            writable = bool(selected) and self._can_write(int(selected[0]))
            editor['entry'].configure(state='normal' if writable else 'disabled')
            editor['button'].configure(state='normal' if writable else 'disabled')
            if marker_changed:
                self._select(page)

    def set_words(self, address, values):
        for offset, value in enumerate(values):
            register = address + offset
            self.words[register] = int(value) & 0xFFFF
            for page, tree in self.tables.items():
                if tree.exists(str(register)):
                    row = list(tree.item(str(register), 'values'))
                    word = self.words[register]
                    row[3:] = (display_register_value(register, word), str(word), f'0x{word:04X}')
                    tree.item(str(register), values=row)
        # A high/low pair retains its raw words while displaying its signed value.
        for high in _MONITOR_PAIRS:
            if address <= high and high + 1 < address + len(values):
                tree = self.tables['M0']
                if tree.exists(str(high)):
                    value = (self.words[high] << 16) | self.words[high + 1]
                    if value & 0x80000000:
                        value -= 0x100000000
                    row = list(tree.item(str(high), 'values'))
                    row[3] = f'{value} counts（合并高低字）'
                    tree.item(str(high), values=row)

    def clear(self):
        self.words.clear()
        for tree in self.tables.values():
            for key in tree.get_children():
                row = list(tree.item(key, 'values'))
                row[3:] = ('—', '—', '—')
                tree.item(key, values=row)
        for editor in self.editors.values():
            editor['value'].set('')
        self.message.set('当前读回已清空；连接后手动读取。')
