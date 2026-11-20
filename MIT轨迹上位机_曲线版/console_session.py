"""One serial worker for control, tuning and explicitly requested upgrades."""
from dataclasses import asdict, is_dataclass
from pathlib import Path
import queue
import threading
import hashlib

from protocol_services import ProtocolSession
from mit_trajectory import MitTrajectoryAbort, MitTrajectoryMixin


def plain(value):
    if is_dataclass(value):
        return plain(asdict(value))
    if isinstance(value, Path):
        return str(value)
    if isinstance(value, bytes):
        return value.hex(' ').upper()
    if isinstance(value, dict):
        return {str(k): plain(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [plain(v) for v in value]
    return value


class ConsoleSession(MitTrajectoryMixin, ProtocolSession):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.firmware_pending = False
        self.firmware_busy = False
        self.manual_run = False
        self.firmware_cancel = threading.Event()

    def log(self, event, **fields):
        super().log(event, **fields)
        if event in ('wire', 'legacy_wire'):
            self.send('wire', plain(fields))

    def _firmware_emit(self, kind, data):
        value = plain(data)
        self.log('firmware', kind=kind, data=value)
        self.send('firmware_event', {'kind': kind, 'data': value})

    def firmware_validate(self, payload):
        self._firmware_job('validate', payload)

    def firmware_upgrade(self, payload):
        self._firmware_job('upgrade', payload)

    def firmware_dry_run(self, payload):
        self._firmware_job('dry_run', payload)

    def firmware_enter(self, payload):
        self._firmware_job('enter', payload)

    def firmware_info(self, payload):
        self._firmware_job('read_info', payload)

    def _firmware_job(self, operation, payload):
        """Runs synchronously in the existing worker; never starts another port thread."""
        if (self.active is not None or self.service_motion is not None
                or self.mit_trajectory is not None or self.manual_run):
            self.firmware_pending = False
            self.firmware_cancel.clear()
            self.send('firmware_error', {'operation': operation, 'error': '先结束运行/校准，再操作升级；控制连接保持不变'})
            self.send('firmware_state', {'busy': False, 'operation': operation})
            return
        self.firmware_busy = True
        self.firmware_pending = False
        self.send('firmware_state', {'busy': True, 'operation': operation})
        try:
            import firmware_upgrade as backend
            # Discard pending ordinary jobs; none may run after the UART format changes.
            discarded = 0
            while True:
                try:
                    self.jobs.get_nowait()
                    discarded += 1
                except queue.Empty:
                    break
            if discarded:
                self.note(f'升级独占串口，已清除{discarded}个待处理普通操作。')
            path = str(payload.get('path', ''))
            if operation == 'validate':
                result = backend.validate(path)
            else:
                if self.rtu is None:
                    raise RuntimeError('请先仅开串口；升级工具不会自动连接')
                port, node = self.rtu.port, self.rtu.node
                work = getattr(backend, operation, None)
                if not callable(work):
                    raise RuntimeError('当前后端尚未提供这项操作，请使用完整升级流程')
                cancel = lambda: self.firmware_cancel.is_set() or self.shutdown.is_set() or self.urgent_stop.is_set()
                if operation in ('enter', 'read_info'):
                    result = work(port, node, self._firmware_emit, cancel)
                else:
                    expected = payload.get('expected_sha256')
                    if not expected or hashlib.sha256(Path(path).read_bytes()).hexdigest() != expected:
                        raise ValueError('HEX已变化，必须重新校验并确认；未开始传输')
                    options = {'expected_sha256': expected}
                    if operation == 'upgrade':
                        options['allow_legacy'] = payload.get('allow_legacy') is True
                    result = work(port, node, path, self._firmware_emit, cancel, **options)
            result = plain(result)
            self.log('firmware_completed', operation=operation, result=result)
            self.send('firmware_done', {'operation': operation, 'result': result})
        except Exception as error:
            self.log('firmware_failed', operation=operation, error=str(error))
            self.send('firmware_error', {'operation': operation, 'error': str(error)})
        finally:
            # Even failures can leave RAM mode/1 Mbps active. Never resume old RTU jobs.
            if operation != 'validate' and self.rtu is not None:
                port = self.rtu.port
                self.rtu = None
                self.verified = False
                try:
                    port.close()
                except OSError as error:
                    self.note(f'烧录操作结束，但系统未确认串口已释放：{error}')
                self.send('disconnected', None)
            self.firmware_busy = False
            self.firmware_pending = False
            self.firmware_cancel.clear()
            self.send('firmware_state', {'busy': False, 'operation': operation})

    def worker(self):
        """The only thread that may access the serial object."""
        while not self.shutdown.is_set():
            begin = self.clock()
            try:
                if self.urgent_stop.is_set():
                    if self.rtu and self.verified:
                        self.stop()
                    else:
                        while not self.jobs.empty():
                            self.jobs.get_nowait()
                        self.note('尚未核对控制协议，未发送控制帧；需要时请现场停机。')
                    self.urgent_stop.clear()
                try:
                    command, payload = self.jobs.get_nowait()
                except queue.Empty:
                    command = None
                if command:
                    firmware = command.startswith('firmware_')
                    if self.mit_trajectory is not None and command not in ('stop', 'disconnect'):
                        self.firmware_pending = False
                        if firmware:
                            self.firmware_cancel.clear()
                            self.send('firmware_error', {'operation': command, 'error': '先停止MIT轨迹，再操作升级'})
                            self.send('firmware_state', {'busy': False, 'operation': command})
                        else:
                            self.note('MIT轨迹独占中：请先停止，不插入普通操作。')
                    elif (self.firmware_pending or self.firmware_busy) and not firmware:
                        self.note('升级独占中：已忽略普通控制或查询任务。')
                    else:
                        if command != 'connect' and not firmware and not self.rtu:
                            raise RuntimeError('请先仅开串口，读取参数后才能控制')
                        if (self.rtu and not firmware and command != 'disconnect'
                                and self.connection_info.get('baudrate', 3000000) != 3000000):
                            raise RuntimeError('当前是1Mbps升级连接，禁止发送控制协议；先断开再开3Mbps控制串口')
                        getattr(self, command)(payload)
                if self.mit_trajectory is not None:
                    self.tick_mit_trajectory()
                elif self.rtu and self.verified and not self.firmware_pending:
                    if self.service_motion:
                        self.poll_service()
                    if self.active:
                        if self.clock()-self.active['started'] >= self.active['spec']['duration']:
                            self.stop('单轮时长已到')
                        else:
                            self.check_trial(self.read_status())
            except Exception as error:
                self.firmware_pending = False
                self.note('操作未完成：'+str(error))
                mit_active = self.mit_trajectory is not None
                stop_confirmed = False
                if (self.active or self.service_motion or mit_active or self.manual_run) and self.rtu:
                    try:
                        self.stop('异常：'+str(error))
                        stop_confirmed = True
                    except Exception as stop_error:
                        self.note('无法确认停机，请现场停止或断电：'+str(stop_error))
                retain_connection = (mit_active and isinstance(error, MitTrajectoryAbort)
                                     and stop_confirmed and self.rtu is not None and self.verified)
                if mit_active:
                    result = {'error': str(error), 'error_type': type(error).__name__,
                              'stop_confirmed': stop_confirmed, 'connection_retained': retain_connection}
                    self.log('mit_trajectory_failed', **result)
                    self.send('mit_trajectory_error', result)
                if retain_connection:
                    self.note('本轮MIT轨迹失败，已确认停机，串口保持连接；请检查日志后重新预览。')
                else:
                    if self.rtu:
                        self.rtu.port.close()
                        self.rtu = None
                    self.verified = False
                    self.manual_run = False
                    self.send('disconnected', None)
            self.shutdown.wait(max(0, (.001 if self.mit_trajectory is not None else .02 if self.active or self.service_motion else .1)-(self.clock()-begin)))
        try:
            self.disconnect()
        except Exception as error:
            self.close_error = str(error)
            self.note('退出时无法确认停机，请现场停止或断电：'+str(error))
            if self.rtu:
                self.rtu.port.close()
        self.log('session_closed')
        self.capture.close()
        self.csvfile.close()
