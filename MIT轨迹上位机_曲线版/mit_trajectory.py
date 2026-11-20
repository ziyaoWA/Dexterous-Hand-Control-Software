"""MIT trajectory in the console's existing worker; no second serial thread.

FC06 speed and position writes are sequential, individually acknowledged and
NOT atomic. Firmware 1113/1114 has no reliable stream-loss watchdog: physical loss
of the cable cannot be guaranteed to stop the motor. The operator must remain
able to stop power. No F1 watchdog parameter or EEPROM is changed here.
"""
from dataclasses import asdict
import json
import math
import uuid

from trajectory_core import plan_trajectory, preview_trajectory
from tuning_core import CSV_FIELDS, crc16, decode, signed, supports_mit
from protocol_services import validate_word


class MitTrajectoryAbort(RuntimeError):
    """Local scheduling/snapshot guard; not an uncertain serial transaction.

    The worker may retain the control connection only after STOP echoes and a
    fresh stopped-state readback have both succeeded. ACK/CRC/write errors keep
    their original I/O exception types and still close the connection.
    """


class MitTrajectoryMixin:
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.mit_trajectory = None

    def assert_idle(self):
        if self.mit_trajectory is not None:
            raise RuntimeError('MIT轨迹运行或保持中，请先停止，再改参数、保存或开始其他操作')
        return super().assert_idle()

    def preview_mit_trajectory(self, payload):
        if not self.state:
            raise RuntimeError('请先读取反馈位置；离线预览可直接调用纯规划函数')
        self.send('mit_trajectory_preview', preview_trajectory(self.state['position_actual_deg'], payload))

    def start_mit_trajectory(self, payload):
        if not self.verified or not self.rtu or not supports_mit((self.identity or {}).get('marker')):
            raise ValueError('MIT轨迹只支持已连接并读回确认的1113/1114固件')
        if self.firmware_pending or self.firmware_busy:
            raise RuntimeError('升级任务独占串口，不能启动MIT轨迹')
        if getattr(self.rtu.port, 'baudrate', 3000000) != 3000000:
            raise ValueError('MIT轨迹需要3Mbps控制连接')
        self.assert_idle()
        self.read_parameters()
        if self.read_registers(0x6024, 1)[0] != (self.identity or {}).get('marker'):
            raise ValueError('当前设备新读回版本与已识别的1113/1114版本不一致，未发送MIT配置或RUN')
        state = self.require_stopped()
        if (state['fault'] or state['electric_error'] or state['voltage_error'] or
                state['calibration_step'] != 10 or state['position_flags'] & 0x37 != 1):
            raise ValueError('先完成校准、排除故障并确认位置反馈就绪')
        plan = plan_trajectory(state['position_actual_deg'], payload)
        if self.urgent_stop.is_set() or self.shutdown.is_set():
            raise InterruptedError('启动前已请求停止，未发送RUN')
        selected = {}
        for key, register in (('kp_q15', 4), ('kd_q15', 5), ('iff_ma', 3), ('current_limit_ma', 13)):
            value = payload.get(key)
            if value is None:value = signed(self.f0[register]) if register == 3 else self.f0[register]
            if isinstance(value, bool) or float(value) != int(value):
                raise ValueError(key+'必须是整数')
            selected[register] = validate_word(register, int(value), (self.identity or {}).get('marker'))
        if selected[13] == 0:
            raise ValueError('峰值电流上限为0，禁止启动MIT轨迹')
        # STOP and every configuration ACK/readback precede the single RUN.
        self.rtu.stop();self.require_stopped()
        configuration = {**selected, 0: 3, 1: 0, 2: round(plan.start_deg*100)&65535,
                         12: plan.max_motor_rpm}
        for register, value in configuration.items():
            self.write_register(register, value)
        self.read_parameters()
        if any(self.f0[register] != value for register, value in configuration.items()):
            raise IOError('MIT配置读回不一致，未发送RUN')
        final_state = self.require_stopped()
        if abs(final_state['position_actual_deg']-state['position_actual_deg']) > .1:
            raise ValueError('配参期间起点变化超过0.1°，未发送RUN；请重新读取并预览轨迹')
        state = final_state
        identifier = 'mit_trajectory_'+uuid.uuid4().hex[:8]
        port = self.rtu.port
        started = self.clock()
        trajectory = {'id': identifier, 'plan': plan, 'selected': selected,
                      'started': started, 'next_due': started, 'phase': 'running',
                      'pending': None, 'point': None, 'point_index': 0, 'hold_until': None,
                      'next_feedback': started+.1, 'feedback_part': 0, 'feedback_values': None,
                      'feedback_started': None,
                      'last_feedback': started, 'samples': [], 'timings': [], 'points_sent': 0,
                      'timeouts': {'timeout': getattr(port, 'timeout', None),
                                   'write_timeout': getattr(port, 'write_timeout', None)},
                      'baseline': state.copy(), 'f0': self.f0.copy(), 'f1': self.f1.copy(),
                      'position_bounds_deg': [max(-327.68,min(plan.start_deg,plan.target_deg)-5.),
                                              min(327.67,max(plan.start_deg,plan.target_deg)+5.)],
                      'current_limit_ma': selected[13], 'board_watchdog_enabled': False}
        self.mit_trajectory = trajectory  # Own the possible RUN before sending it.
        try:
            self.write_register(0x7000, 0)
            self.write_register(0x7000, 1)
            port.timeout = 0
            port.write_timeout = .02
            started = self.clock()
            trajectory['started'] = trajectory['next_due'] = trajectory['last_feedback'] = started
            trajectory['next_feedback'] = started+.1
            self.log('mit_trajectory_started', id=identifier, preview=preview_trajectory(plan.start_deg, payload),
                     f0=self.f0, warning='1113/1114无可靠板端失联停机，FC06位置/速度非原子，现场需可停止或断电')
            self.send('mit_trajectory_state', {'active': True, 'phase': 'running', 'id': identifier,
                      'duration': plan.duration, 'hold_seconds': plan.hold_seconds,
                      'position_bounds_deg': trajectory['position_bounds_deg'],
                      'board_watchdog_enabled': False, 'atomic': False})
        except Exception:
            self.stop('MIT启动确认失败，可能已执行RUN')
            raise

    def _mit_begin_exchange(self, kind, address, value):
        trajectory = self.mit_trajectory
        if self.urgent_stop.is_set() or self.shutdown.is_set():
            return
        node = self.rtu.node
        if kind.startswith('feedback'):
            body = bytes((node, 3))+address.to_bytes(2, 'big')+value.to_bytes(2, 'big')
        else:
            body = bytes((node, 6))+address.to_bytes(2, 'big')+(value&65535).to_bytes(2, 'big')
        frame = body+crc16(body)
        self.rtu.port.reset_input_buffer()
        before = self.clock()
        if kind == 'feedback0':trajectory['feedback_started'] = before
        written = self.rtu.port.write(frame)
        if written is not None and written != len(frame):
            raise IOError('MIT轨迹串口写入不完整；可能已执行部分命令')
        trajectory['pending'] = {'kind': kind, 'address': address, 'value': value, 'body': body,
                                 'buffer': bytearray(), 'started': before, 'deadline': before+.05}
        self.log('mit_trajectory_tx', id=trajectory['id'], kind=kind, tx=frame.hex(' ').upper())

    def _mit_poll_exchange(self):
        trajectory = self.mit_trajectory
        pending = trajectory['pending']
        # We only know when an ACK is observed, not when USB received it. A late
        # observation cannot prove the 50ms deadline was met, even if buffered.
        if self.clock() >= pending['deadline']:
            raise TimeoutError('MIT设定/反馈50ms内未确认有效ACK；可能已执行，停止且不重发RUN')
        available = self.rtu.port.in_waiting
        if available:
            pending['buffer'].extend(self.rtu.port.read(min(available, 256)))
        reply = pending['buffer']
        if len(reply) >= 3:
            expected = 5 if reply[1]&0x80 else (reply[2]+5 if pending['body'][1] == 3 else 8)
            if len(reply) >= expected:
                frame = bytes(reply)
                if (len(frame) != expected or frame[0] != self.rtu.node or
                        frame[-2:] != crc16(frame[:-2]) or frame[1] != pending['body'][1]):
                    raise IOError('MIT回包CRC、站号、功能码或长度不符；不重发过期点')
                if frame[1] == 6 and frame[:-2] != pending['body']:
                    raise IOError('MIT设定写入ACK不符；可能已执行，停止轨迹')
                if frame[1] == 3 and frame[2] != pending['value']*2:
                    raise IOError('MIT反馈回包长度不符')
                self.log('mit_trajectory_ack', id=trajectory['id'], kind=pending['kind'],
                         rx=frame.hex(' ').upper(), elapsed_ms=(self.clock()-pending['started'])*1000)
                trajectory['pending'] = None
                return pending, frame
        return None

    def tick_mit_trajectory(self):
        trajectory = self.mit_trajectory
        if trajectory is None or self.urgent_stop.is_set() or self.shutdown.is_set():
            return
        now = self.clock()
        if trajectory['phase'] == 'holding' and now >= trajectory['hold_until']:
            self.stop('MIT轨迹结束保持时间已到')
            return
        if now-trajectory['last_feedback'] > .4:
            raise MitTrajectoryAbort('MIT反馈超过400ms未更新，停止轨迹')
        feedback_completed = False
        if trajectory['pending']:
            result = self._mit_poll_exchange()
            if result is None:return
            pending, frame = result
            if pending['kind'] == 'velocity':
                if not self.urgent_stop.is_set() and not self.shutdown.is_set():
                    self._mit_begin_exchange('position', 2, trajectory['point']['position_word'])
                return
            if pending['kind'] == 'position':
                self._mit_point_complete()
            else:
                feedback_completed = True
                words = [int.from_bytes(frame[i:i+2], 'big') for i in range(3, len(frame)-2, 2)]
                if pending['kind'] == 'feedback0':
                    trajectory['feedback_values'] = words
                    trajectory['feedback_part'] = 1
                else:
                    self._mit_feedback(trajectory['feedback_values']+words)
                    trajectory['feedback_part'] = 0
                    trajectory['next_feedback'] = self.clock()+.1
            if self.mit_trajectory is None:return
        now = self.clock()
        # A 20ms requested point period can be shorter than two ACKs plus the
        # Windows scheduler quantum. Give overdue telemetry a slot after each
        # complete p/v pair. After one feedback page ACK, a due p/v pair gets the
        # next slot before feedback1. This bounds both kinds of work even when
        # every 16ms ACK is observed on the second 15.625ms scheduler quantum.
        # Only the complete two-page snapshot refreshes the 400ms feedback guard.
        point_due = now >= trajectory['next_due']
        feedback_due = trajectory['feedback_part'] or now >= trajectory['next_feedback']
        if point_due and now-trajectory['next_due'] > .1:
            raise MitTrajectoryAbort('MIT轨迹调度落后超过100ms，停止而不追发过期点')
        if feedback_due and not (feedback_completed and point_due):
            if trajectory['feedback_part'] == 0:self._mit_begin_exchange('feedback0', 0x6000, 20)
            else:self._mit_begin_exchange('feedback1', 0x6014, 9)
        elif point_due:
            lag = now-trajectory['next_due']
            point = trajectory['plan'].sample(now-trajectory['started'])
            point.update(position_word=round(point['position_des_deg']*100)&65535,
                         velocity_word=round(point['velocity_des_rpm'])&65535,
                         due=trajectory['next_due'], submitted=now, lag_ms=lag*1000)
            trajectory['point'] = point
            self._mit_begin_exchange('velocity', 1, point['velocity_word'])

    def _mit_point_complete(self):
        trajectory = self.mit_trajectory
        point = trajectory['point']
        now = self.clock()
        if now-point['due'] > .1:
            raise MitTrajectoryAbort('MIT位置/速度ACK完成已落后100ms，停止而不补发旧点')
        timing = {'elapsed': now-trajectory['started'], 'submitted': point['submitted'],
                  'ack_completed': now, 'lag_ms': point['lag_ms'],
                  'pair_ack_ms': (now-point['submitted'])*1000,
                  'position_des_deg': point['position_des_deg'],
                  'velocity_des_rpm': point['velocity_des_rpm'],
                  'missed_periods': math.floor(max(0., now-point['due'])/trajectory['plan'].period_seconds),
                  'interval_ms': ((now-trajectory['timings'][-1]['ack_completed'])*1000
                                  if trajectory['timings'] else None)}
        trajectory['timings'].append(timing)
        trajectory['points_sent'] += 1
        if point['phase'] == 'holding' and trajectory['phase'] != 'holding':
            trajectory['phase'] = 'holding'
            trajectory['hold_until'] = now+trajectory['plan'].hold_seconds
            self.send('mit_trajectory_state', {'active': True, 'phase': 'holding', 'id': trajectory['id'],
                      'duration': trajectory['plan'].duration, 'hold_seconds': trajectory['plan'].hold_seconds,
                      'board_watchdog_enabled': False})
        self.send('mit_trajectory_progress', {**{k: point[k] for k in ('position_des_deg','velocity_des_rpm','phase')},
                  **timing, 'duration': trajectory['plan'].duration, 'id': trajectory['id'],
                  'points_sent': trajectory['points_sent']})
        self.log('mit_trajectory_point', id=trajectory['id'], **timing)
        period = trajectory['plan'].period_seconds
        trajectory['next_due'] = trajectory['started']+(math.floor((now-trajectory['started'])/period)+1)*period
        trajectory['point'] = None

    def _mit_feedback(self, values):
        trajectory = self.mit_trajectory
        # Compact two-page telemetry preserves 32-bit position counts; PID-only
        # diagnostics are not polled while the serial port streams MIT points.
        values = values+[0]*8+[0]*8+[trajectory['current_limit_ma']]
        now = self.clock()
        span_ms = max(0., now-trajectory['feedback_started'])*1000
        state = decode(values, now, span_ms, trajectory['id'])
        state['effective_current_limit_ma'] = trajectory['current_limit_ma']*state['ocp_scale_q12']//4096
        self.state = state
        self.csv.writerow({key: state.get(key, '') for key in CSV_FIELDS});self.csvfile.flush()
        trajectory['samples'].append({**state, 'elapsed': now-trajectory['started']})
        trajectory['last_feedback'] = now
        self.log('status', values=values, decoded=state, telemetry='MIT two-page compact')
        self.send('status', state.copy())
        reason = None
        if state['fault'] or state['electric_error'] or state['voltage_error']:reason = 'MIT轨迹检测到设备故障'
        elif state['position_flags'] & 0x37 != 1:reason = 'MIT轨迹位置反馈失效'
        elif not -327.68 <= state['position_actual_deg'] <= 327.67:reason = 'MIT轨迹实际位置超出指令可表示范围'
        elif not trajectory['position_bounds_deg'][0] <= state['position_actual_deg'] <= trajectory['position_bounds_deg'][1]:
            reason = 'MIT轨迹实际位置超出起终点外5°的行程包络'
        elif state['mode'] != 3:reason = 'MIT轨迹模式被改变'
        elif abs(state['speed_actual_rpm']) > min(6000, max(500, trajectory['plan'].max_motor_rpm*1.5+100)):
            reason = 'MIT轨迹超过速度停止阈值'
        elif state['current_feedback_magnitude_ma'] > trajectory['current_limit_ma']*1.5:
            reason = 'MIT轨迹电流反馈异常'
        if reason:self.stop(reason)

    def stop(self, reason='操作者点击立即停止'):
        if not isinstance(reason, str):reason = '操作者请求停止'
        trajectory, self.mit_trajectory = self.mit_trajectory, None
        if trajectory is None:return super().stop(reason)
        for name, value in trajectory['timeouts'].items():
            setattr(self.rtu.port, name, value)
        confirmed = False
        try:
            super().stop(reason)
            confirmed = True
        finally:
            path = self.run_dir/(trajectory['id']+'.json')
            report = {key: value for key, value in trajectory.items()
                      if key not in ('plan','pending','point','timeouts')}
            report.update(plan=asdict(trajectory['plan']), reason=reason, stop_confirmed=confirmed)
            report['unfinished_point'] = trajectory['point']
            pending = trajectory['pending']
            report['pending_request'] = ({key: pending[key] for key in ('kind','address','value','started','deadline')}
                                         if pending is not None else None)
            path.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding='utf-8')
            self.log('mit_trajectory_stopped', id=trajectory['id'], reason=reason, stop_confirmed=confirmed, path=str(path))
            self.send('mit_trajectory_state', {'active': False, 'phase': 'stopped', 'id': trajectory['id'],
                      'reason': reason, 'stop_confirmed': confirmed, 'path': str(path),
                      'board_watchdog_enabled': False})

    def disconnect(self, unused=None):
        if self.mit_trajectory is not None:self.stop('断开连接前停止MIT轨迹')
        return super().disconnect(unused)

    def protocol(self, payload):
        if self.mit_trajectory is not None:
            if payload.get('action') == 'write' and payload.get('address') == 0x7000 and payload.get('value') == 8:
                return self.stop('完整协议页停止MIT轨迹')
            raise RuntimeError('MIT轨迹运行或保持中，请先停止；不插入普通读写')
        return super().protocol(payload)
