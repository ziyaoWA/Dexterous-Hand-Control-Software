"""Operator-controlled Modbus tuning for the new finger_joint position-PID 0x1108 image."""
from __future__ import annotations

import csv
import copy
from dataclasses import dataclass
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import queue
import statistics
import threading
import time
import uuid

ROOT = Path(__file__).resolve().parent
MARKER = 0x1111
SUPPORTED_MARKERS = (0x1108, 0x110A, 0x110B, MARKER, 0x1112, 0x1113, 0x1114)
MIT_MARKERS = (0x1113, 0x1114)
POSITION_HOLD_MARKERS = (0x1114,)


def supports_mit(marker):
    """Only a firmware identity explicitly built with MIT may run mode 3."""
    return marker in MIT_MARKERS


def supports_position_hold(marker):
    """Do not treat a legacy reserved register or PID bit as a hold capability."""
    return marker in POSITION_HOLD_MARKERS


def position_loop_summary(data, configured_limit=None):
    """Command-domain limit and its OCP estimate must remain distinct."""
    holding = data.get('mode') == 2 and bool(data.get('position_hold_active', False))
    return (f"{'到位保持中' if holding else '未进入到位保持'}；"
            f"运行设定限流{configured_limit if configured_limit is not None else '—'}mA / "
            f"当前命令域上限{data.get('applied_current_limit_ma','—')}mA / "
            f"降额后上限估算{data.get('effective_current_limit_ma','—')}mA；"
            f"降额{data.get('ocp_scale_q12',0)}/4096；"
            f"位置P/I/D {data.get('position_p_rpm',0)}/{data.get('position_i_rpm',0)}/{data.get('position_d_rpm',0)}rpm；"
            f"速度积分{data.get('speed_integral_ma',0)}mA；PID状态0x{data.get('position_pid_flags',0):03X}")


def signed(value):
    return value - 65536 if value >= 32768 else value


def crc16(data):
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ (0xA001 if crc & 1 else 0)
    return crc.to_bytes(2, 'little')


@dataclass(frozen=True)
class Parameter:
    register: int
    label: str
    minimum: int
    maximum: int
    group: str
    scale: str


PARAMETERS = (
    Parameter(9, '电流环 Kp', 1, 32767, '电流环', 'Q12 原始值'),
    Parameter(10, '电流环 Ki', 1, 32767, '电流环', 'Q12 原始值；0会回落默认值'),
    Parameter(6, '速度环 Kp', 0, 32767, '速度环', 'Q12 原始值'),
    Parameter(7, '速度环 Ki', 0, 32767, '速度环', 'Q12/1ms；0可关闭积分'),
    Parameter(8, '位置环 Kp', 1, 32767, '位置环', 'Q15 rpm/count'),
    Parameter(18, '位置环 Ki', 0, 32767, '位置环', 'Q25 rpm/(count·1ms)；0关闭'),
    Parameter(19, '位置环 Kd', 0, 32767, '位置环', 'Q12；测量速度阻尼；0关闭'),
    Parameter(12, '速度上限 rpm', 30, 5000, '限幅与ESO', '电机轴转速'),
    Parameter(13, '峰值电流上限 mA', 0, 1212, '限幅与ESO', '相电流峰值；0禁止驱动'),
    Parameter(14, 'ESO模型增益 b0×10', 0, 32767, '限幅与ESO', '0使用固件默认'),
    Parameter(15, 'ESO带宽 rad/s', 0, 32767, '限幅与ESO', '0使用固件默认'),
    Parameter(16, 'ESO 补偿系数', 0, 1024, '限幅与ESO', 'Q10；0只观测，1024全量补偿'),
)
PARAM_MAP = {item.register: item for item in PARAMETERS}
CSV_FIELDS = ('time_utc', 'monotonic', 'trial', 'mode', 'speed_reference_rpm',
              'speed_target_rpm', 'speed_actual_rpm', 'position_target_deg',
              'position_actual_deg', 'current_target_ma', 'current_feedback_magnitude_ma',
              'voltage_v', 'motor_request', 'motor_status', 'fault', 'electric_error',
              'voltage_error', 'ocp_scale_q12', 'position_flags', 'sample_span_ms')
CSV_FIELDS += ('calibration_step',)
CSV_FIELDS += ('position_error_counts', 'position_p_rpm', 'position_i_rpm',
               'position_d_rpm', 'position_raw_rpm', 'position_pid_flags',
               'speed_integral_ma', 'applied_current_limit_ma')
CSV_FIELDS += ('effective_current_limit_ma',)
CSV_FIELDS += ('position_hold_active',)


def check_write(register, value, marker=None):
    if not isinstance(value, int) or isinstance(value, bool):
        raise ValueError('寄存器值必须是整数')
    if register in PARAM_MAP:
        p = PARAM_MAP[register]
        valid = p.minimum <= value <= p.maximum
    elif register == 0:
        valid = value in (0, 1, 2) or value == 3 and supports_mit(marker)
    elif register == 1:
        valid = -5000 <= value <= 5000
    elif register == 2:
        valid = -30000 <= value <= 30000
    elif register == 3:
        valid = -200 <= value <= 200
    elif register == 0x7000:
        valid = value in (0, 1, 8)
    elif register == 0x7001:
        valid = value == 1
    else:
        valid = False
    if not valid:
        raise ValueError(f'不允许写寄存器0x{register:04X}={value}')


class RTU:
    def __init__(self, port, node, log, interrupted=lambda: False):
        self.port, self.node, self.log = port, node, log
        self.interrupted = interrupted

    def transact(self, body, count=None, bypass=False):
        if not bypass and self.interrupted():
            raise InterruptedError('操作者已请求停止')
        frame = body + crc16(body)
        self.port.reset_input_buffer()
        self.port.write(frame)
        self.port.flush()
        reply = bytearray()
        deadline = time.monotonic() + .25
        while time.monotonic() < deadline:
            reply.extend(self.port.read(self.port.in_waiting or 1))
            if len(reply) >= 3:
                expected = 5 if reply[1] & 0x80 else (reply[2] + 5 if body[1] == 3 else 8)
                if len(reply) >= expected:
                    break
        valid = (len(reply) >= 5 and reply[0] == self.node and
                 reply[-2:] == crc16(reply[:-2]))
        self.log('wire', tx=frame.hex(' ').upper(), rx=reply.hex(' ').upper(), valid_crc=valid)
        if not valid:
            raise IOError('回包超时、站号不符或CRC错误')
        if reply[1] == body[1] | 0x80:
            raise IOError(f'Modbus异常码{reply[2]}')
        if body[1] == 3:
            if reply[:3] != bytes((self.node, 3, 2*count)) or len(reply) != 5+2*count:
                raise IOError('读取回包功能码/长度不符')
            return [int.from_bytes(reply[3+2*i:5+2*i], 'big') for i in range(count)]
        expected=body[:4]+bytes(2) if body[1]==0x41 else body
        if reply[:-2] != expected:
            raise IOError('写应答与请求不符')

    def read(self, register, count, bypass=False):
        if (register, count) not in ((0, 20), (0x100, 16), (0x6000, 20),
                                      (0x6014, 9), (0x601D, 8), (0x6025, 9), (0x7000, 2)):
            raise ValueError('不允许的读取范围')
        return self.transact(bytes((self.node, 3)) + register.to_bytes(2, 'big') +
                             count.to_bytes(2, 'big'), count, bypass)

    def write(self, register, value, bypass=False):
        check_write(register, value)
        return self.transact(bytes((self.node, 6)) + register.to_bytes(2, 'big') +
                             (value & 65535).to_bytes(2, 'big'), bypass=bypass)

    def stop(self):
        errors = []
        for command in (0, 8):
            try:
                self.write(0x7000, command, bypass=True)
                time.sleep(.025)
            except Exception as error:
                errors.append(str(error))
        if errors:
            raise IOError('停止应答未确认，请现场停止或断电：' + '; '.join(errors))


def decode(values, timestamp, span_ms, trial=None):
    result = dict(zip(CSV_FIELDS, (
        datetime.now(timezone.utc).isoformat(), timestamp, trial or '', values[1],
        signed(values[2]), signed(values[4]), signed(values[13]), signed(values[17])/100,
        signed(values[14])/100, signed(values[5]), values[12], values[11]/100,
        values[6], values[7], values[20], values[18], values[19], values[25],
        values[26], round(span_ms, 3), values[9])))
    if len(values) >= 46:
        def signed32(high,low):
            value=(high << 16)|low
            return value-4294967296 if value & 2147483648 else value
        result['position_actual_deg']=signed32(values[27],values[28])*28125/(5269*512*100)
        result.update(position_error_counts=signed32(values[37],values[38]),
                      position_p_rpm=signed(values[39]), position_i_rpm=signed(values[40]),
                      position_d_rpm=signed(values[41]), position_raw_rpm=signed(values[42]),
                      position_pid_flags=values[43], speed_integral_ma=signed(values[44]),
                      applied_current_limit_ma=values[45])
        result['position_hold_active'] = values[1] == 2 and bool(values[43] & 0x0100)
    else:
        result.update({name:0 for name in CSV_FIELDS[21:]})
    return result


def response_metrics(samples, mode, target, initial):
    """Metrics for a commanded step, using actual timestamps and the pre-step baseline."""
    if not samples:
        return {'sample_count': 0, 'insufficient_samples': True}
    metric_mode = 2 if mode == 3 else mode
    if metric_mode not in (0, 1, 2):
        raise ValueError('响应统计仅支持电流、速度、位置和MIT模式')
    key = ('current_feedback_magnitude_ma', 'speed_actual_rpm', 'position_actual_deg')[metric_mode]
    if mode == 0:
        target, initial = abs(target), abs(initial)
    delta = target-initial
    times = [s['elapsed'] for s in samples]
    ys = [s[key] for s in samples]
    tail_start = times[-1] - max(.1, (times[-1]-times[0])*.2)
    tail = [y for y, t in zip(ys, times) if t >= tail_start]
    direction = 1 if delta >= 0 else -1
    progress = [(y-initial)*direction for y in ys]
    amplitude = abs(delta)
    tolerance = max(amplitude*.05, (10, 30, .1)[metric_mode])
    settle = None
    if len(samples) >= 5 and times[-1]-times[0] >= .2:
        last_bad = max((i for i, y in enumerate(ys) if abs(y-target) > tolerance), default=-1)
        candidate = last_bad+1
        if candidate < len(times) and times[-1]-times[candidate] >= .2:
            settle = round(times[candidate], 4)
    def crossing(fraction):
        return next((t for t, p in zip(times, progress) if p >= amplitude*fraction), None)
    t10, t90 = crossing(.1), crossing(.9)
    gaps = [b-a for a, b in zip(times, times[1:])]
    return {
        'sample_count': len(samples), 'insufficient_samples': len(samples) < 5,
        'commanded_step_target': target, 'initial_feedback': initial,
        'tail_mean': round(statistics.mean(tail), 4),
        'tail_error_target_minus_mean': round(target-statistics.mean(tail), 4),
        'overshoot_percent': round(max(0, max(progress)-amplitude)/amplitude*100, 3) if amplitude else None,
        'rise_10_90_seconds': round(t90-t10, 4) if amplitude and t10 is not None and t90 is not None else None,
        'settling_5_percent_seconds': settle, 'settling_absolute_tolerance': tolerance,
        'peak_current_magnitude_ma': max(s['current_feedback_magnitude_ma'] for s in samples),
        'current_request_limit_fraction': round(sum(abs(s['current_target_ma']) >= s.get('current_limit_ma', 700)*.98
                                                  for s in samples)/len(samples), 4),
        'max_sample_gap_ms': round(max(gaps)*1000, 3) if gaps else None,
        'actual_sample_rate_hz': round((len(times)-1)/(times[-1]-times[0]), 2) if gaps and times[-1] > times[0] else None,
        'interpretation': ('MIT模式按目标位置观察整机响应，同时受目标速度、阻尼及前馈电流影响；不代表独立位置环收敛。'
                           if mode == 3 else '相对用户阶跃的整机响应，包含固件指令滤波与ESO；不是电流环高频带宽测试。'),
    }


def validate_test(spec, f0, state):
    mode = int(spec['mode'])
    rotary_bench = spec.get('rotary_bench', False)
    if not isinstance(rotary_bench, bool):
        raise ValueError('台架连续旋转选项须为明确的布尔值')
    numbers = ('target', 'duration', 'speed_cap', 'speed_guard', 'travel_guard')
    if mode not in (0, 1, 2) or not all(math.isfinite(float(spec[k])) for k in numbers):
        raise ValueError('测试参数无效')
    if not .2 <= spec['duration'] <= 50:
        raise ValueError('单轮最长50秒；超过10秒仅用于低速计圈测试')
    speed_limit = f0[12] or 5000
    current_limit_value = float(spec.get('test_current_limit_ma', f0[13]))
    if not math.isfinite(current_limit_value) or not current_limit_value.is_integer():
        raise ValueError('本轮峰值电流上限须填写整数mA')
    current_limit = int(current_limit_value)
    if not 1 <= current_limit <= 1212 or not 30 <= speed_limit <= 5000:
        raise ValueError('本轮限流须为1～1212mA；0mA禁止启动。请先读取并设置限幅')
    max_travel = 1080 if rotary_bench else 180
    absolute_position_limit = 3600 if rotary_bench else 300
    if not 100 <= spec['speed_guard'] <= 6000 or not .5 <= spec['travel_guard'] <= max_travel:
        raise ValueError(f'速度停止阈值100～6000rpm；本模式最大偏移0.5～{max_travel}°')
    if rotary_bench and not (mode == 1 and 500 <= abs(spec['target']) <= 2000
                             and spec['duration'] <= 4 and current_limit <= 700
                             and spec['speed_guard'] <= 3000):
        raise ValueError('连续台架仅限速度模式500～2000rpm、最长4秒、限流≤700mA、超速阈值≤3000rpm')
    if spec['duration'] > 10 and (mode != 1 or abs(spec['target']) > 100
                                 or current_limit > 300 or spec['speed_guard'] > 300):
        raise ValueError('10～50秒仅用于速度模式计圈：目标≤100rpm、限流≤300mA、超速停止阈值≤300rpm')
    if state['position_flags'] & 0x37 != 1 or abs(state['position_actual_deg']) > absolute_position_limit:
        raise ValueError(f'位置观测器未就绪或超出本模式±{absolute_position_limit}°范围')
    target = float(spec['target'])
    if mode == 0:
        if not 1 <= abs(target) <= min(200, current_limit):
            raise ValueError('电流模式单次给定范围±1～200mA，且不得超过当前限幅')
        if not 30 <= spec['speed_cap'] <= min(speed_limit, spec['speed_guard']):
            raise ValueError('电流模式速度上限须在30rpm与全局上限/停止阈值之间')
        target = round(target)
    elif mode == 1:
        if not 30 <= abs(target) <= min(speed_limit, spec['speed_guard']):
            raise ValueError('速度给定须在30rpm与当前限幅/停止阈值之间')
        target = round(target)
    else:
        if not .1 <= abs(target) <= spec['travel_guard']:
            raise ValueError('位置增量至少0.1°，且不得超过本轮偏移停止阈值')
        if not 30 <= spec['speed_cap'] <= min(speed_limit, spec['speed_guard'], 5000):
            raise ValueError('位置模式速度上限超出范围')
        target = round(state['position_actual_deg']+target, 2)
        if abs(target) > 300:
            raise ValueError('最终位置超出±300°，本轮不跨位置回绕边界')
    return {**spec, 'mode': mode, 'target_effective': target, 'current_limit_ma': current_limit,
            'absolute_position_limit_deg': absolute_position_limit}


class Session:
    def __init__(self, send, root=ROOT, clock=time.monotonic):
        self.root, self.clock, self.send = Path(root), clock, send
        self.run_dir = self.root/'sessions'/(datetime.now().strftime('%Y%m%d_%H%M%S_%f')+'_'+uuid.uuid4().hex[:6])
        self.run_dir.mkdir(parents=True)
        self.capture = (self.run_dir/'capture.jsonl').open('a', encoding='utf-8', buffering=1)
        self.csvfile = (self.run_dir/'samples.csv').open('w', encoding='utf-8-sig', newline='')
        self.csv = csv.DictWriter(self.csvfile, fieldnames=CSV_FIELDS)
        self.csv.writeheader()
        self.jobs, self.urgent_stop, self.shutdown = queue.Queue(), threading.Event(), threading.Event()
        self.rtu = None
        self.verified = False
        self.connection_info = None
        self.pending_power_cycle = False
        self.identity = None
        self.original = None
        self.f0 = self.f1 = self.state = None
        self.active = None
        self.plan = None
        self.close_error = None
        self.trial_count = 0

    def log(self, event, **fields):
        self.capture.write(json.dumps({'time_utc': datetime.now(timezone.utc).isoformat(),
                                       'monotonic': self.clock(), 'event': event, **fields}, ensure_ascii=False)+'\n')

    def note(self, text):
        self.log('note', message=text)
        self.send('note', text)

    def connect(self, payload):
        if self.rtu:
            return
        import serial
        node, portname = int(payload['node']), str(payload['port'])
        if not 1 <= node <= 5:
            raise ValueError('站号范围1～5')
        baudrate = int(payload.get('baudrate', 3000000))
        if baudrate not in (1000000, 3000000):
            raise ValueError('控制串口3Mbps；升级串口1Mbps')
        port = serial.Serial(port=None, baudrate=baudrate, bytesize=8, parity='N', stopbits=1,
                             xonxoff=False, rtscts=False, dsrdtr=False, timeout=.01, write_timeout=.2)
        # Match the user's known-working JCOM profile (UI inspected 2026-11-06):
        # DTR checked, RTS unchecked. This is a confirmed configuration
        # difference, not yet proof of the reported power/current symptom.
        port.dtr, port.rts = True, False
        port.port = portname
        try:
            self.log('serial_open_attempt', port=portname, baudrate=baudrate, dtr=True, rts=False,
                     format='8N1', flow_control=False)
            port.open()
            candidate = RTU(port, node, self.log, lambda: self.urgent_stop.is_set() or self.shutdown.is_set())
            self.rtu, self.verified = candidate, False
            self.connection_info = {'port': portname, 'node': node, 'marker': MARKER, 'baudrate': baudrate}
            self.pending_power_cycle = bool(payload.get('power_cycled'))
            self.log('serial_port_opened_without_query', identity=self.connection_info, tx_bytes=0)
            self.send('port_opened', self.connection_info.copy())
            self.note('串口已打开，发送0字节；空闲不自动查询。需要时手动点“②读取参数”。')
        except Exception:
            port.close()
            self.rtu = None
            raise

    def read_parameters(self, unused=None):
        f0, f1 = self.rtu.read(0, 20), self.rtu.read(0x100, 16)
        if not self.verified:
            diag = self.rtu.read(0x601D, 8)
            if f0[11] != self.connection_info['node'] or diag[7] not in SUPPORTED_MARKERS:
                raise ValueError('未识别到配套控制固件（0x1111/0x1112/0x1113/0x1114或兼容历史版本），禁止控制')
            self.connection_info['marker'] = diag[7]
            if self.identity != self.connection_info:
                self.identity, self.original = self.connection_info.copy(), {'f0': f0.copy(), 'f1': f1.copy()}
                (self.run_dir/'original_parameters.json').write_text(json.dumps(
                    {**self.identity, **self.original}, ensure_ascii=False, indent=2), encoding='utf-8')
            self.verified = True
            self.log('connected_parameters', identity=self.identity, f0=f0, f1=f1)
        self.f0, self.f1 = f0, f1
        self.log('parameter_readback', f0=self.f0, f1=self.f1)
        self.send('parameters', self.f0)
        self.read_status()
        if self.pending_power_cycle:
            self.pending_power_cycle = False
            self.verify_saved()

    def read_once(self, unused=None):
        if not self.verified:
            raise RuntimeError('先手动读取参数并核对固件')
        self.read_status()

    def read_status(self, bypass=False):
        begin = self.clock()
        values = self.rtu.read(0x6000, 20, bypass) + self.rtu.read(0x6014, 9, bypass)
        extra=self.rtu.read(0x6025,9,bypass)
        values += [0]*8+extra
        end = self.clock()
        self.state = decode(values, end, (end-begin)*1000,
                            self.active['id'] if self.active else None)
        self.state['effective_current_limit_ma']=(min(
            self.f0[13], self.state['applied_current_limit_ma'])*self.state['ocp_scale_q12'])//4096
        self.log('status', values=values, decoded=self.state)
        self.csv.writerow(self.state)
        self.csvfile.flush()
        self.send('status', self.state.copy())
        return self.state

    def require_stopped(self, bypass=False):
        if not self.verified:
            raise RuntimeError('尚未读取验证固件，请先点“②读取参数”')
        state = self.read_status(bypass)
        if state['motor_request'] not in (0, 1) or state['motor_status'] & 15 or abs(state['speed_actual_rpm']) > 60:
            raise RuntimeError('电机尚未停稳，请先点立即停止，等转速下降后再操作')
        return state

    def assert_idle(self):
        if self.active or getattr(self,'service_motion',None):
            raise RuntimeError('测试期间禁止改参数、恢复或保存')
        self.require_stopped()

    def wait_stopped(self):
        deadline = self.clock()+2
        while True:
            state = self.read_status(bypass=True)
            if (state['motor_request'] in (0, 1) and not state['motor_status'] & 15
                    and abs(state['speed_actual_rpm']) <= 60):
                return
            if self.clock() >= deadline:
                raise RuntimeError('停止后2秒仍未确认停稳，请现场停止或断电')
            time.sleep(.04)

    def apply(self, changes):
        self.assert_idle()
        changes = {int(k): int(v) for k, v in changes.items()}
        if not changes or any(k not in PARAM_MAP for k in changes):
            raise ValueError('参数组无效')
        for register, value in changes.items():
            check_write(register, value)
        self.rtu.stop()
        self.require_stopped()
        before = self.rtu.read(0, 20)
        for register, value in changes.items():
            self.rtu.write(register, value)
        self.read_parameters()
        if any(self.f0[r] != v for r, v in changes.items()):
            raise IOError('写入后参数读回不一致')
        if any(self.f0[i] != before[i] for i in range(20) if i not in {*changes, 17}):
            raise IOError('检测到本轮以外的F0参数发生变化')
        self.log('parameters_applied_ram', changes=changes, f0=self.f0)
        self.note('临时参数已读回一致；未写入掉电存储。下一轮启动时固件加载这些增益。')

    def restore(self, unused=None):
        if not self.original:
            raise RuntimeError('没有本次连接的原始快照')
        self.apply({r: self.original['f0'][r] for r in PARAM_MAP})
        self.note('已恢复首次连接的调参项，未保存；控制模式与目标值由下一轮测试明确设置。')

    def apply_plan(self, plan):
        self.assert_idle()
        if not isinstance(plan, dict) or not plan.get('id') or not isinstance(plan.get('parameters'), dict):
            raise ValueError('试验计划格式无效')
        if any(plan.get(k) != self.identity[k] for k in ('port', 'node', 'marker')):
            raise ValueError('试验计划的串口、站号或固件标志与当前连接不符')
        changes = {int(k): int(v) for k, v in plan['parameters'].items()}
        if not changes or any(r not in PARAM_MAP for r in changes):
            raise ValueError('计划只能修改已支持的调参项')
        for r, v in changes.items():
            check_write(r, v)
        if 13 in changes and int(plan['test'].get('test_current_limit_ma', changes[13])) != changes[13]:
            raise ValueError('计划参数与测试项的电流上限不一致')
        future = self.rtu.read(0, 20)
        for r, v in changes.items():
            future[r] = v
        validate_test(plan['test'], future, self.state)
        self.apply(changes)
        self.plan = copy.deepcopy(plan)
        self.plan['consumed'] = False
        self.log('plan_applied_readback_verified', plan=self.plan)
        self.send('plan_applied', copy.deepcopy(self.plan))
        self.note('本轮计划参数已写入并读回。请观察计划说明，再本人点击③启动；参数未掉电保存。')

    def exit_plan(self, unused=None):
        self.assert_idle()
        self.plan = None
        self.log('operator_exited_plan')
        self.send('plan_applied', None)

    def check_plan(self, spec):
        if not self.plan:
            return
        if self.plan.get('consumed'):
            raise ValueError('本计划已启动过一轮；需重新应用计划后才能重复测试')
        differences = [f'F0-{int(r):02} 期望{v}、实际{self.f0[int(r)]}'
                       for r, v in self.plan['parameters'].items() if self.f0[int(r)] != int(v)]
        for k, v in self.plan['test'].items():
            if k not in spec or abs(float(spec[k])-float(v)) > 1e-9:
                differences.append(f'测试项{k}与计划不一致')
        if differences:
            raise ValueError('本轮未启动：'+'；'.join(differences)+'。请重新应用计划。')

    @property
    def pending_path(self):
        return self.root/'pending_save.json'

    def save(self, unused=None):
        self.assert_idle()
        self.rtu.stop()
        self.require_stopped()
        self.read_parameters()
        if self.f0[0] not in (0, 1, 2) and not (self.f0[0] == 3 and supports_mit((self.identity or {}).get('marker'))):
            raise ValueError('保存前退出校准或不受当前固件支持的模式；MIT需要0x1113或0x1114')
        expected = {**self.identity, 'f0': self.f0.copy(), 'f1': self.f1.copy(),
                    'time_utc': datetime.now(timezone.utc).isoformat()}
        self.rtu.write(0x7001, 1)
        self.pending_path.write_text(json.dumps(expected, ensure_ascii=False, indent=2), encoding='utf-8')
        self.log('save_requested_not_yet_verified', expected=expected)
        self.send('save_state', '已发送保存请求，待断电重上电核验')
        self.note('已发H7001保存请求。固件未返回存储写入结果，重上电读回一致后才显示保存验证成功。')

    def verify_saved(self):
        if not self.pending_path.exists():
            self.note('没有待核验的保存记录。')
            return
        expected = json.loads(self.pending_path.read_text(encoding='utf-8'))
        if any(expected[k] != self.identity[k] for k in ('port', 'node', 'marker')):
            raise ValueError('待保存记录对应其它连接/站号，不能核验')
        differences = [f'F0-{i:02}' for i in range(20) if i != 17 and expected['f0'][i] != self.f0[i]]
        differences += [f'F1-{i:02}' for i in range(16) if expected['f1'][i] != self.f1[i]]
        self.log('operator_confirmed_reboot_save_check', differences=differences,
                 f0=self.f0, f1=self.f1)
        if differences:
            self.send('save_state', '保存核验不一致：'+'、'.join(differences))
            self.note('重上电参数与保存请求不一致；未显示保存成功。')
        else:
            self.send('save_state', '保存验证成功：重上电F0/F1读回一致')
            self.note('保存验证成功：操作者确认重上电，F0/F1读回一致（F0-17为实时监控，排除比较）。')
            expected['verified_time_utc'] = datetime.now(timezone.utc).isoformat()
            (self.run_dir/'verified_save.json').write_text(json.dumps(expected, ensure_ascii=False, indent=2), encoding='utf-8')
            self.pending_path.unlink()

    def start(self, spec):
        self.assert_idle()
        self.read_parameters()
        before = self.require_stopped()
        if before['fault'] or before['electric_error'] or before['voltage_error']:
            raise RuntimeError('存在故障，先核实原因；本工具不自动清故障')
        if before['calibration_step'] != 10:
            raise RuntimeError('未识别到已校准状态，拒绝启动普通运动测试')
        self.check_plan(spec)
        spec = validate_test(spec, self.f0, before)
        self.rtu.stop()
        self.require_stopped()
        # Apply the explicitly entered trial limit while stopped, then verify
        # it with the complete command. Current mode must not use spd=0:
        # firmware treats zero as a fallback to the global speed limit.
        self.rtu.write(13, spec['current_limit_ma'])
        for register, value in ((0, spec['mode']), (1, round(spec['speed_cap']) if spec['mode'] in (0, 2) else 0),
                                (2, 0), (3, 0)):
            self.rtu.write(register, value)
        command_register = (3, 1, 2)[spec['mode']]
        value = round(spec['target_effective']*(100 if spec['mode'] == 2 else 1))
        self.rtu.write(command_register, value)
        readback = self.rtu.read(0, 20)
        if (readback[0] != spec['mode'] or signed(readback[command_register]) != value
                or readback[13] != spec['current_limit_ma']
                or (spec['mode'] in (0, 2) and readback[1] != round(spec['speed_cap']))):
            raise IOError('目标/模式读回不符，拒绝启动')
        if any(readback[r] != self.f0[r] for r in PARAM_MAP if r != 13):
            raise IOError('参数在启动准备期间变化，拒绝启动')
        self.f0 = readback.copy()
        self.send('parameters', self.f0.copy())
        self.trial_count += 1
        self.active = {'id': f'trial_{self.trial_count:03}', 'spec': spec, 'f0': readback,
                       'f1': self.f1.copy(), 'baseline': before, 'started': self.clock(), 'samples': [],
                       'plan_id': self.plan['id'] if self.plan else None}
        if self.plan:
            self.plan['consumed'] = True
        self.log('operator_started_test', trial=self.active['id'], spec=spec, f0=readback, f1=self.f1,
                 plan_id=self.active['plan_id'])
        # active is set first: an uncertain RUN ACK must still trigger STOP cleanup.
        self.rtu.write(0x7000, 1)
        self.send('trial_started', {k: v for k, v in self.active.items() if k != 'samples'})
        self.note(f"开始{self.active['id']}，仅发送一次RUN；随后查询，{spec['duration']}秒后停止，不自动回起点。")

    def check_trial(self, state):
        if not self.active:
            return
        trial, reason = self.active, None
        spec = trial['spec']
        elapsed = self.clock()-trial['started']
        trial['samples'].append({**state, 'elapsed': elapsed, 'current_limit_ma': spec['current_limit_ma']})
        if state['fault'] or state['electric_error'] or state['voltage_error']:
            reason = '检测到故障或电气/电压异常'
        elif state['current_feedback_magnitude_ma'] > spec['current_limit_ma']*1.5:
            reason = '实际峰值电流超过本轮限幅的1.5倍'
        elif abs(state['speed_actual_rpm']) > spec['speed_guard']:
            reason = '反馈转速超过本轮停止阈值'
        elif abs(state['position_actual_deg']-trial['baseline']['position_actual_deg']) > spec['travel_guard']:
            reason = '位置偏移超过本轮停止阈值'
        elif (abs(state['position_actual_deg']) > spec['absolute_position_limit_deg']
              or state['position_flags'] & 0x37 != 1):
            reason = '位置反馈未就绪/跳变/接近回绕边界'
        elif elapsed > .3 and state['mode'] != spec['mode']:
            reason = '运行模式被外部改变'
        elif elapsed >= spec['duration']:
            reason = '单轮时长已到'
        if reason:
            self.stop(reason)

    def finish_trial(self, reason, stopped):
        if not self.active:
            return
        trial, self.active = self.active, None
        mode = trial['spec']['mode']
        baseline_key = ('current_feedback_magnitude_ma', 'speed_actual_rpm', 'position_actual_deg')[2 if mode == 3 else mode]
        trial.update({'stop_reason': reason, 'stop_state_confirmed': stopped,
                      'metrics': response_metrics(trial['samples'], mode, trial['spec']['target_effective'],
                                                  trial['baseline'][baseline_key])})
        path = self.run_dir/(trial['id']+'.json')
        path.write_text(json.dumps(trial, ensure_ascii=False, indent=2), encoding='utf-8')
        self.log('test_finished', trial=trial['id'], reason=reason, stopped=stopped, metrics=trial['metrics'])
        self.send('trial_finished', {'path': str(path), **trial})

    def stop(self, reason='操作者点击立即停止'):
        while True:
            try:
                self.jobs.get_nowait()
            except queue.Empty:
                break
        self.log('stop_requested', reason=reason)
        stopped = False
        try:
            self.rtu.stop()
            self.wait_stopped()
            stopped = True
            self.note('停止指令及停机状态已确认：'+reason)
        finally:
            self.finish_trial(reason, stopped)

    def disconnect(self, unused=None):
        if self.rtu:
            if self.active or getattr(self,'service_motion',None):
                self.stop('断开连接')
            self.rtu.port.close()
            self.rtu = None
            self.verified = False
            self.send('disconnected', None)
            self.note('已断开；可断电重上电后勾选核验再连接。')

    def worker(self):
        while not self.shutdown.is_set():
            begin = self.clock()
            try:
                if self.urgent_stop.is_set():
                    if self.rtu and self.verified:
                        self.stop()
                    else:
                        while not self.jobs.empty():
                            self.jobs.get_nowait()
                        self.note('尚未核对通讯协议，未发送控制帧；需要时请现场停机。')
                    self.urgent_stop.clear()
                try:
                    command, payload = self.jobs.get_nowait()
                except queue.Empty:
                    command = None
                if command:
                    if command != 'connect' and not self.rtu:
                        raise RuntimeError('请先连接并读取')
                    getattr(self, command)(payload)
                if self.rtu and self.verified and getattr(self,'service_motion',None):
                    self.poll_service()
                if self.rtu and self.verified and self.active:
                    if self.clock()-self.active['started'] >= self.active['spec']['duration']:
                        self.stop('单轮时长已到')
                    else:
                        state = self.read_status()
                        self.check_trial(state)
            except Exception as error:
                self.note('操作未完成：'+str(error))
                if (self.active or getattr(self,'service_motion',None)) and self.rtu:
                    try:
                        self.stop('异常：'+str(error))
                    except Exception as stop_error:
                        self.note('无法确认停机，请现场停止或断电：'+str(stop_error))
                if self.rtu:
                    self.rtu.port.close()
                    self.rtu = None
                self.verified = False
                self.send('disconnected', None)
            self.shutdown.wait(max(0, (.02 if self.active or getattr(self,'service_motion',None) else .1)-(self.clock()-begin)))
        try:
            self.disconnect()
        except Exception as error:
            self.close_error = str(error)
            self.log('close_stop_failed', error=str(error))
            self.send('note', '退出时无法确认停机，请现场停止或断电：'+str(error))
            if self.rtu:
                self.rtu.port.close()
        self.log('session_closed')
        self.capture.close()
        self.csvfile.close()
