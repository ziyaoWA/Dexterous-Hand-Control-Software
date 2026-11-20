"""Pure trapezoidal/triangular MIT trajectory planning; never accesses a port."""
from dataclasses import asdict, dataclass
import math

FIRMWARE_GEAR_RATIO = 52.69


def finite_number(value, name):
    if isinstance(value, bool):
        raise ValueError(name+'必须是有效数字')
    try:
        result = float(value)
    except (TypeError, ValueError):
        raise ValueError(name+'必须是有效数字') from None
    if not math.isfinite(result):
        raise ValueError(name+'必须是有限数字')
    return result


@dataclass(frozen=True)
class Trajectory:
    start_deg: float
    target_deg: float
    max_motor_rpm: int
    gear_ratio: float
    period_seconds: float
    hold_seconds: float
    accel_seconds: float
    decel_seconds: float
    cruise_seconds: float
    peak_output_deg_s: float
    acceleration_deg_s2: float
    deceleration_deg_s2: float
    duration: float
    direction: int

    def sample(self, elapsed):
        elapsed = max(0., finite_number(elapsed, '轨迹时间'))
        ta, tc, td = self.accel_seconds, self.cruise_seconds, self.decel_seconds
        peak = self.peak_output_deg_s
        if elapsed >= self.duration or self.direction == 0:
            return {'time': min(elapsed, self.duration), 'position_des_deg': self.target_deg,
                    'velocity_des_rpm': 0., 'phase': 'holding'}
        if elapsed < ta:
            distance = .5*self.acceleration_deg_s2*elapsed*elapsed
            speed = self.acceleration_deg_s2*elapsed
            phase = 'accelerating'
        elif elapsed < ta+tc:
            distance = .5*peak*ta+peak*(elapsed-ta)
            speed, phase = peak, 'cruising'
        else:
            remain = self.duration-elapsed
            distance = abs(self.target_deg-self.start_deg)-.5*self.deceleration_deg_s2*remain*remain
            speed, phase = self.deceleration_deg_s2*remain, 'decelerating'
        return {'time': elapsed, 'position_des_deg': self.start_deg+self.direction*distance,
                'velocity_des_rpm': self.direction*speed*self.gear_ratio/6., 'phase': phase}


def plan_trajectory(start_deg, payload):
    start = finite_number(start_deg, '起点角度')
    target = finite_number(payload.get('target_deg'), '绝对目标角度')
    speed = finite_number(payload.get('max_motor_rpm'), '最高电机转速')
    accel = finite_number(payload.get('accel_seconds'), '加速时间')
    decel = finite_number(payload.get('decel_seconds', accel), '减速时间')
    gear = finite_number(payload.get('gear_ratio', FIRMWARE_GEAR_RATIO), '减速比')
    period = finite_number(payload.get('period_seconds', .02), '发送周期')
    hold = finite_number(payload.get('hold_seconds', 5), '结束保持时间')
    if abs(gear-FIRMWARE_GEAR_RATIO) > 1e-8:
        raise ValueError('1113/1114固件的位置换算固定52.69，轨迹减速比必须与固件一致')
    if not -327.68 <= start <= 327.67 or not -327.68 <= target <= 327.67:
        raise ValueError('MIT指令的绝对起点与目标须在−327.68～327.67°内，不截断或回绕')
    if speed != int(speed) or not 1 <= speed <= 5000:
        raise ValueError('最高电机转速须为1～5000rpm整数')
    if not .02 <= accel <= 60 or not .02 <= decel <= 60:
        raise ValueError('加速、减速时间须在0.02～60秒内')
    if not .01 <= period <= .1 or not 0 <= hold <= 60:
        raise ValueError('发送周期10～100ms；结束保持0～60秒，默认20ms与5秒')
    # Match the actual position words used on the wire.
    start, target = round(start, 2), round(target, 2)
    distance = abs(target-start)
    vmax = speed*6./gear
    acceleration, deceleration = vmax/accel, vmax/decel
    if distance == 0:
        ta = td = tc = peak = duration = 0.
    elif distance >= .5*vmax*(accel+decel):
        ta, td, peak = accel, decel, vmax
        tc = distance/vmax-.5*(ta+td)
        duration = ta+tc+td
    else:
        peak = math.sqrt(2.*distance/(1./acceleration+1./deceleration))
        ta, td, tc = peak/acceleration, peak/deceleration, 0.
        duration = ta+td
    if duration > 600:
        raise ValueError('轨迹运动时间超过600秒，请调整行程或速度')
    direction = 1 if target > start else -1 if target < start else 0
    return Trajectory(start, target, int(speed), gear, period, hold, ta, td, tc,
                      peak, acceleration, deceleration, duration, direction)


def preview_trajectory(start_deg, payload):
    plan = plan_trajectory(start_deg, payload)
    steps = min(3000, max(1, math.ceil(plan.duration/plan.period_seconds)))
    times = {0., plan.accel_seconds, plan.accel_seconds+plan.cruise_seconds, plan.duration}
    times.update(plan.duration*i/steps for i in range(steps+1))
    return {**asdict(plan), 'profile': 'trapezoidal' if plan.cruise_seconds else 'triangular',
            'points': [plan.sample(t) for t in sorted(times)],
            'atomic': False, 'transport': 'FC06单播：先速度ACK，再位置ACK，RUN一次',
            'board_watchdog_enabled': False}
