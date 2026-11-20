"""Tk-only reference/physical feedback plots, updated on the Tk UI thread."""
import math
import tkinter as tk
from tkinter import ttk


class TrajectoryPlot(ttk.Frame):
    REFERENCE_COLOR = '#2463ab'
    FEEDBACK_COLOR = '#cd641d'

    def __init__(self, parent):
        super().__init__(parent)
        self._reference = []
        self._feedback = []
        self._planned_end = 0.0
        self._redraw_job = None
        self.show_reference_position = tk.BooleanVar(self, value=True)
        self.show_feedback_position = tk.BooleanVar(self, value=True)
        self.show_reference_speed = tk.BooleanVar(self, value=True)
        self.show_feedback_speed = tk.BooleanVar(self, value=True)
        controls = ttk.Frame(self, padding=(4, 2))
        controls.grid(row=0, column=0, sticky='ew')
        for text, variable in (
                ('规划位置', self.show_reference_position), ('实测位置', self.show_feedback_position),
                ('规划速度', self.show_reference_speed), ('实测速度', self.show_feedback_speed)):
            ttk.Checkbutton(controls, text=text, variable=variable).pack(side='left', padx=(0, 12))
            variable.trace_add('write', lambda *_: self._schedule_redraw())
        self.canvases = []
        for row in range(2):
            canvas = tk.Canvas(self, height=145, background='white',
                               highlightthickness=1, highlightbackground='#d4dce8')
            canvas.grid(row=row + 1, column=0, sticky='nsew', pady=(0 if row == 0 else 4, 0))
            canvas.bind('<Configure>', self._schedule_redraw, add='+')
            self.canvases.append(canvas)
            self.rowconfigure(row + 1, weight=1)
        self.empty_hint = ttk.Label(self, text='勾选规划或实测曲线，即可显示对应的位置 / 速度图。',
                                    anchor='center', padding=12)
        self.columnconfigure(0, weight=1)
        self._schedule_redraw()

    @staticmethod
    def _number(value, name):
        if isinstance(value, bool):
            raise ValueError(name + '须为有限数值')
        try:
            number = float(value)
        except (TypeError, ValueError) as error:
            raise ValueError(name + '须为有限数值') from error
        if not math.isfinite(number):
            raise ValueError(name + '须为有限数值')
        return number

    def show_plan(self, preview: dict):
        """Replace the reference and clear feedback; never mutate ``preview``."""
        reference = []
        planned_end = 0.0
        if preview is not None:
            duration = self._number(preview['duration'], '运动时长')
            hold = self._number(preview.get('hold_seconds', 0), '保持时长')
            if duration < 0 or hold < 0:
                raise ValueError('运动和保持时长不能为负数')
            for point in preview.get('points', ()):
                elapsed = self._number(point['time'], '参考时间')
                if elapsed < 0:
                    raise ValueError('参考时间不能为负数')
                reference.append((elapsed,
                                  self._number(point['position_des_deg'], '参考位置'),
                                  self._number(point['velocity_des_rpm'], '参考转速')))
            reference.sort(key=lambda point: point[0])
            planned_end = max(duration, reference[-1][0] if reference else 0.0) + hold
            if reference and hold > 0:
                # The planner's terminal sample has v=0; hold the same position.
                terminal_position = reference[-1][1]
                if reference[-1][0] < duration:
                    reference.append((duration, terminal_position, 0.0))
                reference.append((planned_end, terminal_position, 0.0))
        self._reference = reference
        self._feedback = []
        self._planned_end = planned_end
        self._schedule_redraw()

    def add_feedback(self, elapsed: float, position_deg: float, speed_rpm: float):
        """Append one physical sample with its actual elapsed timestamp."""
        point = (self._number(elapsed, '反馈时间'),
                 self._number(position_deg, '反馈位置'),
                 self._number(speed_rpm, '反馈转速'))
        if point[0] < 0:
            raise ValueError('反馈时间不能为负数')
        self._feedback.append(point)
        self._schedule_redraw()

    def _schedule_redraw(self, _event=None):
        if self._redraw_job is None:
            self._redraw_job = self.after_idle(self._redraw)

    @staticmethod
    def _tick(value):
        if abs(value) < 1e-10:
            value = 0.0
        return f'{value:.3g}'

    def _redraw(self):
        self._redraw_job = None
        extent = max([self._planned_end, 0.0] +
                     [point[0] for point in self._reference] +
                     [point[0] for point in self._feedback])
        xmax = extent if extent > 0 else 1.0
        # Sorting is visual only; received samples retain their original order.
        feedback = sorted(self._feedback, key=lambda point: point[0])
        selections = ((self.show_reference_position.get(), self.show_feedback_position.get()),
                      (self.show_reference_speed.get(), self.show_feedback_speed.get()))
        any_plot = any(any(selection) for selection in selections)
        if any_plot:
            self.empty_hint.grid_remove()
        else:
            self.empty_hint.grid(row=3, column=0, sticky='nsew')
        self.rowconfigure(3, weight=0 if any_plot else 1)
        for index, canvas in enumerate(self.canvases, start=1):
            reference_visible, feedback_visible = selections[index - 1]
            visible = reference_visible or feedback_visible
            self.rowconfigure(index, weight=1 if visible else 0)
            if visible:
                canvas.grid()
            else:
                canvas.grid_remove()
                canvas.delete('all')
                continue
            canvas.delete('all')
            width, height = canvas.winfo_width(), canvas.winfo_height()
            title = '输出轴位置 / °' if index == 1 else '电机轴转速 / rpm'
            canvas.create_text(10, 13, anchor='w', text=title, fill='#24384e',
                               font=('Microsoft YaHei UI', 9, 'bold'), tags='title')
            if width < 180 or height < 85:
                continue
            left, right, top, bottom = 65, width - 16, 50, height - 29
            legend_x = max(10, width - 245)
            for offset, label, color, dashed, enabled in (
                    (0, '参考给定', self.REFERENCE_COLOR, True, reference_visible),
                    (113, '实测反馈', self.FEEDBACK_COLOR, False, feedback_visible)):
                if not enabled:
                    continue
                x = legend_x + offset
                options = dict(fill=color, width=2, tags='legend')
                if dashed:
                    options['dash'] = (5, 3)
                canvas.create_line(x, 31, x + 22, 31, **options)
                canvas.create_text(x + 27, 31, anchor='w', text=label,
                                   fill='#495a6c', font=('Microsoft YaHei UI', 9), tags='legend')
            visible_reference = self._reference if reference_visible else []
            visible_feedback = feedback if feedback_visible else []
            values = [point[index] for point in visible_reference + visible_feedback]
            # Position must remain readable near a nonzero operating point.
            if index == 2:
                values.append(0.0)
            low, high = (min(values), max(values)) if values else (-1.0, 1.0)
            padding = max((high - low) * .08, .05 if index == 1 else 1.0)
            low, high = low - padding, high + padding
            canvas.create_rectangle(left, top, right, bottom, outline='#c4d0df', tags='axis')
            for tick in range(4):
                fraction = tick / 3
                x = left + fraction * (right - left)
                y = bottom - fraction * (bottom - top)
                canvas.create_line(x, top, x, bottom, fill='#edf1f6', tags='grid')
                canvas.create_line(left, y, right, y, fill='#edf1f6', tags='grid')
                value = low + fraction * (high - low)
                y_label = f'{value:.2f}' if index == 1 else (
                    f'{value:.0f}' if high - low >= 30 else f'{value:.1f}')
                canvas.create_text(left - 6, y, anchor='e',
                                   text=y_label,
                                   fill='#596b7f', font=('Segoe UI', 8), tags='tick')
                anchor = 'w' if tick == 0 else 'e' if tick == 3 else 'center'
                canvas.create_text(x, bottom + 11, anchor=anchor,
                                   text=self._tick(fraction * xmax),
                                   fill='#596b7f', font=('Segoe UI', 8), tags='tick')
            canvas.create_text(right, height - 7, anchor='e', text='时间 / s',
                               fill='#596b7f', font=('Microsoft YaHei UI', 8), tags='xlabel')
            for points, color, tag, dashed in (
                    (visible_reference, self.REFERENCE_COLOR, 'reference', True),
                    (visible_feedback, self.FEEDBACK_COLOR, 'feedback', False)):
                coordinates = []
                for point in points:
                    coordinates.extend((left + point[0] / xmax * (right - left),
                                        bottom - (point[index] - low) / (high - low) * (bottom - top)))
                if len(coordinates) >= 4:
                    options = dict(fill=color, width=2, tags=tag)
                    if dashed:
                        options['dash'] = (5, 3)
                    canvas.create_line(*coordinates, **options)
                elif coordinates:
                    x, y = coordinates
                    canvas.create_oval(x - 2.5, y - 2.5, x + 2.5, y + 2.5,
                                       fill=color, outline=color, tags=tag)
            if not visible_reference and not visible_feedback:
                canvas.create_text((left + right) / 2, (top + bottom) / 2,
                                   text='生成轨迹后显示参考，运行时叠加实测反馈',
                                   fill='#8a99aa', font=('Microsoft YaHei UI', 9), tags='empty')

    def destroy(self):
        if self._redraw_job is not None:
            try:
                self.after_cancel(self._redraw_job)
            except tk.TclError:
                pass
            self._redraw_job = None
        super().destroy()
