"""Selectable, read-only charts for samples from the M0 monitor group."""

from collections import deque
import math
import tkinter as tk
from tkinter import font, ttk

from monitor_channels import CHANNELS, DEFAULT_CHANNELS


class MonitorPlot(ttk.Frame):
    HISTORY_SIZE = 600
    COLORS = ('#1767a5', '#b94e18', '#137864', '#8d4998', '#376c25', '#a13e52',
              '#475abb', '#93700d', '#457582', '#73443d')

    def __init__(self, parent, start_callback, stop_callback):
        super().__init__(parent, padding=(0, 6, 0, 0))
        self._start_callback = start_callback
        self._history = deque(maxlen=self.HISTORY_SIZE)
        self._started = self._redraw_job = None
        self.interval = tk.StringVar(self, value='200')
        self.status = tk.StringVar(self, value='连接后可开始采样。')
        self.selected = {
            channel.key: tk.BooleanVar(self, value=channel.key in DEFAULT_CHANNELS)
            for channel in CHANNELS
        }
        self._legend_font = font.Font(root=self, family='Microsoft YaHei UI', size=9)

        toolbar = ttk.Frame(self)
        toolbar.grid(row=0, column=0, columnspan=2, sticky='ew', pady=(0, 6))
        controls = ttk.Frame(toolbar)
        controls.pack(anchor='w')
        ttk.Label(controls, text='采样间隔').pack(side='left')
        self.interval_entry = ttk.Entry(controls, width=7, textvariable=self.interval)
        self.interval_entry.pack(side='left', padx=(5, 4))
        ttk.Label(controls, text='ms').pack(side='left', padx=(0, 10))
        self.start_button = ttk.Button(controls, text='开始采样', command=self._start)
        self.start_button.pack(side='left', padx=(0, 4))
        self.stop_button = ttk.Button(controls, text='停止采样', command=stop_callback)
        self.stop_button.pack(side='left')
        ttk.Label(toolbar, textvariable=self.status, wraplength=680).pack(anchor='w', pady=(3, 0))

        selector = ttk.LabelFrame(self, text='选择曲线', padding=(4, 2))
        selector.grid(row=1, column=0, sticky='ns', padx=(0, 7))
        choices = tk.Canvas(selector, width=194, height=190, highlightthickness=0)
        scroll = ttk.Scrollbar(selector, orient='vertical', command=choices.yview)
        choices.configure(yscrollcommand=scroll.set)
        choices.pack(side='left', fill='both', expand=True)
        scroll.pack(side='right', fill='y')
        menu = ttk.Frame(choices)
        menu_id = choices.create_window(0, 0, window=menu, anchor='nw')
        menu.bind('<Configure>', lambda _: choices.configure(scrollregion=choices.bbox('all')))
        choices.bind('<Configure>', lambda event: choices.itemconfigure(menu_id, width=event.width))
        self._bind_wheel(choices, choices)
        self._bind_wheel(menu, choices)
        for channel in CHANNELS:
            option = ttk.Checkbutton(menu, text=f'{channel.code}  {channel.label}',
                                     variable=self.selected[channel.key])
            option.pack(fill='x', pady=1)
            self._bind_wheel(option, choices)
            self.selected[channel.key].trace_add('write', lambda *_: self._schedule_redraw())

        plots = ttk.Frame(self)
        plots.grid(row=1, column=1, sticky='nsew')
        self.canvas = tk.Canvas(plots, width=360, height=190, background='white',
                                highlightthickness=1, highlightbackground='#d4dce8')
        plot_scroll = ttk.Scrollbar(plots, orient='vertical', command=self.canvas.yview)
        self.canvas.configure(yscrollcommand=plot_scroll.set)
        self.canvas.pack(side='left', fill='both', expand=True)
        plot_scroll.pack(side='right', fill='y')
        self.canvas.bind('<Configure>', self._schedule_redraw, add='+')
        self._bind_wheel(self.canvas, self.canvas)
        self.rowconfigure(1, weight=1)
        self.columnconfigure(1, weight=1)
        self.set_state(False, False)
        self._schedule_redraw()

    @staticmethod
    def _bind_wheel(widget, canvas):
        widget.bind('<MouseWheel>', lambda event: canvas.yview_scroll(
            -max(1, event.delta // 120) if event.delta > 0 else max(1, -event.delta // 120), 'units'))
        widget.bind('<Button-4>', lambda _: canvas.yview_scroll(-1, 'units'))
        widget.bind('<Button-5>', lambda _: canvas.yview_scroll(1, 'units'))

    def _start(self):
        try:
            period = int(self.interval.get())
            if period <= 0:
                raise ValueError()
        except ValueError:
            self.set_status('请输入正整数采样间隔。')
            return
        self._start_callback(period)

    def set_state(self, available, sampling):
        editable = 'normal' if available and not sampling else 'disabled'
        self.start_button.configure(state=editable)
        self.interval_entry.configure(state=editable)
        self.stop_button.configure(state='normal' if sampling else 'disabled')

    def set_status(self, text):
        self.status.set(text)

    def clear_samples(self):
        self._history.clear()
        self._started = None
        self._schedule_redraw()

    def add_sample(self, timestamp, values):
        """Convert each sample once so changing selections can redraw recent data."""
        if self._started is None:
            self._started = timestamp
        self._history.append((timestamp - self._started,
                              tuple(channel.value(values) for channel in CHANNELS)))
        self._schedule_redraw()

    def _schedule_redraw(self, _event=None):
        if self._redraw_job is None:
            self._redraw_job = self.after_idle(self._redraw)

    def _redraw(self):
        self._redraw_job = None
        canvas = self.canvas
        canvas.delete('all')
        width, height = canvas.winfo_width(), canvas.winfo_height()
        if width < 190 or height < 40:
            return
        bands = {}
        for index, channel in enumerate(CHANNELS):
            if self.selected[channel.key].get():
                # Unrelated diagnostic words share a raw unit but need distinct scales.
                key = (channel.unit, channel.key if channel.unit == '原始值' else '')
                bands.setdefault(key, []).append((index, channel))
        content_height = 0
        if not bands:
            canvas.create_text(width / 2, height / 2, text='勾选左侧参数查看曲线。', fill='#536477')
        graph_height = max(48, min(210, (height - 2) // max(1, len(bands)) - 62))
        for (unit, _), entries in bands.items():
            content_height = self._draw_band(width, content_height, graph_height, unit, entries)
        canvas.configure(scrollregion=(0, 0, width - 2, max(height - 2, content_height)))

    def _draw_band(self, width, start, graph_height, unit, entries):
        canvas = self.canvas
        left, right = 65, width - 16
        canvas.create_text(12, start + 14, text=unit, anchor='w',
                           fill='#24384e', font=('Microsoft YaHei UI', 9, 'bold'))
        x, y = self._legend_font.measure(unit) + 32, start + 14
        for index, channel in entries:
            label = f'{channel.code} {channel.label}'
            item_width = self._legend_font.measure(label) + 38
            if x > 13 and x + item_width > right:
                x, y = 13, y + 19
            color = self.COLORS[index % len(self.COLORS)]
            canvas.create_line(x, y, x + 17, y, fill=color, width=2)
            canvas.create_text(x + 22, y, anchor='w', text=label, font=self._legend_font, fill='#405366')
            x += item_width
        top = y + 16
        bottom = top + graph_height
        history = list(self._history)
        xmin = history[0][0] if history else 0.0
        xmax = max(history[-1][0], xmin + .001) if len(history) > 1 else xmin + 1.0
        series = [(index, [(stamp, readings[index]) for stamp, readings in history
                           if math.isfinite(readings[index])]) for index, _ in entries]
        data = [value for _, points in series for _, value in points]
        low, high = (min(data), max(data)) if data else (0.0, 1.0)
        padding = max((high - low) * .08, .01 if unit in ('°', 'V', 'A') else 1.0)
        low, high = low - padding, high + padding

        canvas.create_rectangle(left, top, right, bottom, outline='#c4d0df')
        for tick in range(4):
            fraction = tick / 3
            grid_x = left + fraction * (right - left)
            grid_y = bottom - fraction * (bottom - top)
            canvas.create_line(grid_x, top, grid_x, bottom, fill='#eaf0f5')
            canvas.create_line(left, grid_y, right, grid_y, fill='#eaf0f5')
            canvas.create_text(left - 6, grid_y, anchor='e', fill='#536477', font=('Segoe UI', 8),
                               text=f'{low + fraction * (high - low):.4g}')
            anchors = ('w', 'center', 'center', 'e')
            canvas.create_text(grid_x, bottom + 10, anchor=anchors[tick], fill='#536477',
                               font=('Segoe UI', 8), text=f'{xmin + fraction * (xmax - xmin):.4g}')
        canvas.create_text(right, bottom + 24, anchor='e', text='时间 / s', fill='#536477',
                           font=('Microsoft YaHei UI', 8))
        if not data:
            canvas.create_text((left + right) / 2, (top + bottom) / 2,
                               text='等待采样', fill='#66798d')
        for index, points in series:
            color = self.COLORS[index % len(self.COLORS)]
            coordinates = []
            for stamp, value in points:
                coordinates.extend((left + (stamp - xmin) * (right - left) / (xmax - xmin),
                                    bottom - (value - low) * (bottom - top) / (high - low)))
            if len(coordinates) >= 4:
                canvas.create_line(*coordinates, fill=color, width=1.5, tags='trace')
            elif coordinates:
                x, y = coordinates
                canvas.create_oval(x - 2, y - 2, x + 2, y + 2, fill=color, outline=color, tags='trace')
        return bottom + 32

    def destroy(self):
        if self._redraw_job is not None:
            self.after_cancel(self._redraw_job)
            self._redraw_job = None
        super().destroy()
