"""Numerical plot channels decoded from the High-81339 M0 register group."""

from dataclasses import dataclass
from typing import Sequence

from tuning_core import signed


# Same output-angle conversion as tuning_core.decode and the firmware position observer.
_DEGREES_PER_COUNT = 28125 / (5269 * 512 * 100)


@dataclass(frozen=True)
class MonitorChannel:
    key: str
    index: int
    label: str
    unit: str
    is_signed: bool = False
    scale: float = 1
    word_count: int = 1

    @property
    def code(self):
        code = f'M0-{self.index:02d}'
        return f'{code}/{self.index + 1:02d}' if self.word_count == 2 else code

    def value(self, words: Sequence[int]) -> int | float:
        high = words[self.index]
        value = signed(high) if self.is_signed else high
        if self.word_count == 2:
            value = (value << 16) | words[self.index + 1]
        return value if self.scale == 1 else value * self.scale


CHANNELS = (
    MonitorChannel('speed_actual_rpm', 13, '实际转速', 'rpm', is_signed=True),
    MonitorChannel('position_actual_deg', 27, '实际位置（连续）', '°',
                   is_signed=True, scale=_DEGREES_PER_COUNT, word_count=2),
    MonitorChannel('current_peak_a', 12, '实际峰值电流', 'A', scale=.001),
    MonitorChannel('speed_target_rpm', 4, '速度请求', 'rpm', is_signed=True),
    MonitorChannel('position_target_deg', 17, '目标位置', '°', is_signed=True, scale=.01),
    MonitorChannel('current_target_a', 5, '电流请求', 'A', is_signed=True, scale=.001),
    MonitorChannel('voltage_v', 11, '母线电压', 'V', scale=.01),
    MonitorChannel('temperature_c', 23, '芯片温度', '℃', is_signed=True),
    MonitorChannel('speed_reference_rpm', 2, '生效速度给定', 'rpm', is_signed=True),
    MonitorChannel('position_reference_deg', 3, '生效位置给定', '°', is_signed=True, scale=.01),
    MonitorChannel('position_word_deg', 14, '位置（16 位饱和值）', '°', is_signed=True, scale=.01),
    MonitorChannel('position_counts', 27, '绝对累计位置', 'counts', is_signed=True, word_count=2),
    MonitorChannel('position_error_counts', 37, '位置误差', 'counts', is_signed=True, word_count=2),
    MonitorChannel('position_p_rpm', 39, '位置环 P 输出', 'rpm', is_signed=True),
    MonitorChannel('position_i_rpm', 40, '位置环 I 输出', 'rpm', is_signed=True),
    MonitorChannel('position_d_rpm', 41, '位置环 D 输出', 'rpm', is_signed=True),
    MonitorChannel('position_raw_rpm', 42, '位置 PID 限幅前输出', 'rpm', is_signed=True),
    MonitorChannel('speed_integral_a', 44, '速度环积分', 'A', is_signed=True, scale=.001),
    MonitorChannel('applied_current_limit_a', 45, '命令域电流上限', 'A', scale=.001),
    MonitorChannel('ocp_percent', 25, '过流降额系数', '%', scale=100 / 4096),
    MonitorChannel('command', 0, '末次命令', '原始值'),
    MonitorChannel('mode', 1, '控制模式', '原始值'),
    MonitorChannel('motor_request', 6, '驱动请求', '原始值'),
    MonitorChannel('motor_status', 7, '驱动状态', '原始值'),
    MonitorChannel('command_count', 8, '命令计数', '次'),
    MonitorChannel('calibration_step', 9, '校准步骤', '原始值'),
    MonitorChannel('stop_reason', 10, '停止原因', '原始值'),
    MonitorChannel('resolver_x', 15, 'Resolver X', 'ADC 原始值'),
    MonitorChannel('resolver_y', 16, 'Resolver Y', 'ADC 原始值'),
    MonitorChannel('electric_error', 18, '电气错误位', '原始值'),
    MonitorChannel('voltage_error', 19, '电压错误位', '原始值'),
    MonitorChannel('fault', 20, '故障码', '原始值'),
    MonitorChannel('fault_level', 21, '故障等级', '原始值'),
    MonitorChannel('comm_idle_ms', 22, '通讯空闲时间', 'ms'),
    MonitorChannel('fault_detail', 24, '故障详情', '原始值'),
    MonitorChannel('position_flags', 26, '位置状态位', '原始值'),
    MonitorChannel('drive_position', 29, '驱动累计位置', '原始值', word_count=2),
    MonitorChannel('drive_target', 31, '驱动目标位置', '原始值', word_count=2),
    MonitorChannel('drive_remaining', 33, '驱动剩余距离', '原始值'),
    MonitorChannel('microstep_index', 34, '微步索引', '原始值'),
    MonitorChannel('drive_stop_reason', 35, '驱动停止原因', '原始值'),
    MonitorChannel('firmware_marker', 36, '固件标识', '原始值'),
    MonitorChannel('position_pid_flags', 43, '位置 PID 状态位', '原始值'),
    MonitorChannel('calibration_diag_46', 46, '校准诊断 46', '原始值'),
    MonitorChannel('calibration_diag_47', 47, '校准诊断 47', '原始值'),
    MonitorChannel('calibration_diag_48', 48, '校准诊断 48', '原始值'),
    MonitorChannel('pwm_ceiling', 49, 'PWM 诊断', '原始值'),
)

DEFAULT_CHANNELS = ('speed_actual_rpm', 'position_actual_deg', 'current_peak_a')
