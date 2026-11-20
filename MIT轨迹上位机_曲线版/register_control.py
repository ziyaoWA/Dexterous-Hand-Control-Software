"""Register-page operations on the existing High-81339 serial worker."""

import hashlib
import json
import time
from datetime import datetime, timezone

from protocol_services import GROUPS
from trajectory_wire import request_frame
from tuning_core import supports_mit, supports_position_hold


def validate_page_word(address, value, marker):
    if type(value) is not int or not -32768 <= value <= 65535:
        raise ValueError('输入必须能表示为一个16位寄存器值')
    word = value & 0xFFFF
    if 0 <= address < 20 and address != 17:
        if address == 0 and word not in (0, 1, 2, 100) and not (word == 3 and supports_mit(marker)):
            raise ValueError('当前固件不支持该控制模式')
        if address == 11 and not 1 <= word <= 5:
            raise ValueError('固件站号范围为1～5')
        if address == 13 and word > 1212:
            raise ValueError('固件峰值限流范围为0～1212 mA')
        if address in (18, 19) and word > 32767:
            raise ValueError('固件位置 Ki/Kd 范围为0～32767')
    elif 0x100 <= address <= 0x10F:
        if address >= 0x10D:
            if not supports_position_hold(marker):
                raise ValueError('当前固件的 F1-13～15 为保留项')
            maximum = 1212 if address == 0x10E else 100
            if word > maximum:
                raise ValueError(f'固件允许的寄存器原始值为0～{maximum}')
    elif address == 0x7000 and word in (0, 1, 5, 8):
        pass
    elif address == 0x7001 and word in (0, 1, 2):
        pass
    else:
        raise ValueError('该寄存器或命令值不支持写入')
    return word


class RegisterControlMixin:
    @property
    def pending_path(self):
        # Keep each serial port/station/firmware's reboot check independent.
        identity = self.identity
        key = json.dumps([identity[field] for field in ('port', 'node', 'marker')])
        return self.root / ('pending_save_' + hashlib.sha256(key.encode()).hexdigest()[:16] + '.json')

    def control_registers(self, payload):
        try:
            if not self.verified or self.mit_trajectory is not None:
                raise ValueError('请在已识别固件、轨迹结束后操作')
            action = payload['action']
            if action == 'read':
                self._read_page(payload['address'], payload['count'])
            elif action == 'write':
                address = payload['address']
                word = validate_page_word(address, payload['value'], self.identity['marker'])
                self._write_page(address, word)
            elif action == 'verify_save':
                self.read_parameters()
                self.verify_saved()
        except (ValueError, InterruptedError) as error:
            self.note(str(error))
        finally:
            self.send('register_done', None)

    def _publish_words(self, address, values):
        self.send('register_words', {'address': address, 'values': values})

    def sample_monitor(self, request):
        result = {'request': request}
        try:
            if not self.verified:
                raise OSError('请先连接并读取驱动器状态')
            result['values'] = self._read_page(0x6000, len(GROUPS['M0'][1]))
            result['time'] = self.clock()
        except InterruptedError:
            result['interrupted'] = True
        except OSError as error:
            self.log('monitor_read_failed', error=str(error))
            result['error'] = str(error)
        self.send('monitor_sample', result)

    def _read_page(self, address, count):
        if count < 1 or not any(base <= address and address + count <= base + len(names)
                                for base, names in GROUPS.values()):
            raise ValueError('读取地址超出固件寄存器组')
        values = []
        for offset in range(0, count, 20):
            values.extend(self.read_registers(address + offset, min(20, count - offset)))
        self.log('register_readback', address=address, values=values)
        self._publish_words(address, values)
        return values

    def _write_page_word(self, address, word):
        self.rtu.transact(request_frame(self.rtu.node, 6, address, word)[:-2])

    def _switch_station(self, station):
        self.rtu.node = station
        self.connection_info['node'] = station
        self.verified = False
        self.send('node_changed', station)
        try:
            self.read_parameters()
        except InterruptedError as error:
            # The address has already changed. Reconnect after the worker handles STOP.
            raise OSError('站号已切换，读回被中断。请按新站号重新连接。') from error

    def _record_save(self, message):
        expected = {**self.identity, 'f0': self.f0.copy(), 'f1': self.f1.copy(),
                    'time_utc': datetime.now(timezone.utc).isoformat()}
        self.pending_path.write_text(json.dumps(expected, ensure_ascii=False, indent=2), encoding='utf-8')
        self.log('save_requested_not_yet_verified', expected=expected)
        self.note(message + '。断电重上电后，在 C0 核验保存。')

    def _write_page(self, address, word):
        if address == 0x7000:
            self._control_command(word)
            return
        if address == 0x7001 and word in (1, 2):
            self._parameter_command(word)
            return
        self._write_page_word(address, word)
        if address == 11 and word != self.rtu.node:
            self._switch_station(word)
            actual = self.f0[11]
        else:
            actual, = self._read_page(address, 1)
            if 0 <= address < 20:
                self.f0[address] = actual
            elif 0x100 <= address < 0x110:
                self.f1[address - 0x100] = actual
        if actual != word:
            self.note(f'0x{address:04X} 写应答已收到，读回 0x{actual:04X}，与请求 0x{word:04X} 不一致。')
        else:
            persistence = '。需掉电保留时请在 C0 保存。' if address < 0x7000 else '。'
            self.note(f'0x{address:04X} 写入并读回 0x{actual:04X}' + persistence)

    def _control_command(self, word):
        if word == 8:
            self.stop('C0 停止')
            return
        if word in (1, 5):
            self._write_page_word(0x7000, 0)
        if word == 1:
            # An interrupted or lost acknowledgement may still leave RUN accepted.
            self.manual_run = True
            self.send('manual_state', True)
        self._write_page_word(0x7000, word)
        self._read_page(0x7000, 1)
        self.read_status()
        self.note({0: '清空命令字已应答。', 1: '固件已应答 RUN。需要停机时点击停止。',
                   5: '清故障命令已应答，监控状态已刷新。'}[word])

    def _parameter_command(self, word):
        self.read_parameters()
        if word == 2 and (self.manual_run or self.state['motor_status'] & 0x0F):
            raise ValueError('请先停止电机，再恢复默认并保存')
        self._write_page_word(0x7001, word)
        if word == 2:
            # The firmware background save task restores defaults and sets address 1.
            time.sleep(.2)
            self._switch_station(1)
        self._read_page(0x7000, 2)
        self._record_save('固件已应答保存请求' if word == 1 else '已读回默认参数并收到保存请求应答')
