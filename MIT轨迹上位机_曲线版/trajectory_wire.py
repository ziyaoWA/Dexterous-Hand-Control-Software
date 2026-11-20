"""Pure RTU frame/inspection helpers. No serial ports, workers or UI objects."""
import math

from protocol_services import F0_NAMES, F1_NAMES, H_NAMES, M0_NAMES
from tuning_core import crc16, signed


MODE_NAMES = {0: '电流模式', 1: '速度模式', 2: '位置模式', 3: 'MIT模式', 100: '校准模式'}
EXCEPTION_NAMES = {
    1: '非法功能码', 2: '非法数据地址', 3: '非法数据值', 4: '设备处理失败',
    5: '请求已接受，处理未完成', 6: '设备忙', 8: '存储器奇偶校验错误',
    10: '网关路径不可用', 11: '网关目标设备未响应',
}


def _integer(value, name, minimum, maximum):
    if isinstance(value, bool) or not isinstance(value, int) or not minimum <= value <= maximum:
        raise ValueError(f'{name}须为{minimum}～{maximum}的整数')
    return value


def request_frame(node, fc, address, value):
    """Build one unicast FC03 or FC06 request; CRC bytes are low then high.

    This is framing, not authorization to run or write any given register.
    FC03 ``value`` is a word count; FC06 accepts signed16 or raw unsigned16.
    """
    _integer(node, '站号', 1, 5)
    _integer(fc, '功能码', 3, 6)
    if fc not in (3, 6):
        raise ValueError('仅支持0x03查询和0x06单寄存器写入')
    _integer(address, '寄存器地址', 0, 65535)
    if fc == 3:
        _integer(value, '查询寄存器数量', 1, 20)
        if address + value > 65536:
            raise ValueError('查询范围超过16位寄存器地址')
    else:
        _integer(value, '寄存器值', -32768, 65535)
    body = bytes((node, fc)) + address.to_bytes(2, 'big') + (value & 65535).to_bytes(2, 'big')
    return body + crc16(body)


def _hex(frame):
    return frame.hex(' ').upper()


def iter_setpoints(plan, node):
    """Yield quantized planning points and their sequential speed/position frames.

    The speed frame must be acknowledged before the position frame at runtime;
    this generator performs no I/O and does not imply atomic p/v updates.
    """
    _integer(node, '站号', 1, 5)
    duration, period = float(plan.duration), float(plan.period_seconds)
    if not math.isfinite(duration) or duration < 0 or not math.isfinite(period) or period <= 0:
        raise ValueError('轨迹时长须非负且发送周期须为正的有限数值')
    steps = math.floor(duration / period)
    for index in range(steps + 1):
        elapsed = index * period
        if elapsed >= duration:
            break
        yield _setpoint(plan, node, elapsed)
    yield _setpoint(plan, node, duration)


def _setpoint(plan, node, elapsed):
    sample = plan.sample(elapsed)
    position = float(sample['position_des_deg'])
    velocity = float(sample['velocity_des_rpm'])
    if not math.isfinite(position) or not math.isfinite(velocity):
        raise ValueError('轨迹采样出现非有限位置或速度')
    if not -327.68 <= position <= 327.67:
        raise ValueError('位置超出有符号0.01°寄存器范围')
    position_word, speed = round(position * 100), round(velocity)
    speed_bytes = request_frame(node, 6, 1, speed)
    position_bytes = request_frame(node, 6, 2, position_word)
    return elapsed, position_word / 100, speed, _hex(speed_bytes), _hex(position_bytes)


def _as_frame(value, name):
    if isinstance(value, (bytes, bytearray)):
        frame = bytes(value)
    elif isinstance(value, str):
        try:
            frame = bytes.fromhex(value)
        except ValueError as error:
            raise ValueError(f'{name}不是有效HEX报文') from error
    else:
        raise ValueError(f'{name}须为HEX字符串或bytes')
    if len(frame) < 4 or frame[-2:] != crc16(frame[:-2]):
        raise ValueError(f'{name}长度不足或CRC16/MODBUS错误')
    return frame


def _label(address):
    for base, names in ((0, F0_NAMES), (0x100, F1_NAMES), (0x6000, M0_NAMES), (0x7000, H_NAMES)):
        if base <= address < base + len(names):
            return names[address - base]
    return '未定义寄存器（原始值）'


def _current(word, magnitude=False):
    value = word if magnitude else signed(word)
    return f'{value} mA / {value / 1000:.3f} A' + ('（峰值幅值）' if magnitude else '')


def _engineering(address, word):
    number = signed(word)
    if address == 0 or address == 0x6001:
        return f'{word}：{MODE_NAMES.get(word, "未知模式")}'
    if address in (1, 0x6002, 0x6004, 0x600D):
        return f'{number} rpm'
    if address in (2, 0x6003, 0x600E, 0x6011):
        suffix = '（16位饱和反馈）' if address == 0x600E else ''
        return f'{number / 100:.2f}°{suffix}'
    if address in (3, 17, 0x6005, 0x602C):
        return _current(word)
    if address in (13, 0x100, 0x10E, 0x600C, 0x602D):
        return _current(word, address == 0x600C)
    if address == 12:
        return f'{word} rpm' + ('（0采用固件默认）' if word == 0 else '')
    if address in (4, 5, 8):
        return f'{word}（Q15原始值）'
    if address in (6, 7, 9, 10, 19):
        return f'{word}（Q12原始值）'
    if address == 18:
        return f'{word}（Q25原始值）'
    if address == 14:
        return f'{word / 10:g} rpm/s/mA' if word else '0（采用固件默认模型增益）'
    if address == 15:
        return f'{word} rad/s' if word else '0（采用固件默认带宽）'
    if address in (16, 0x6019):
        return f'{word}/1024（Q10）' if address == 16 else f'{word}/4096（Q12降额）'
    if address in (0x10D, 0x10F):
        return f'{word / 100:.2f}°' if word else '0（采用默认0.05°）'
    if address in (0x108, 0x109, 0x600B):
        return f'{word / 100:.2f} V'
    if address == 0x6017:
        return f'{number} °C'
    if address in (0x101, 0x102, 0x103, 0x104, 0x10C, 0x6016):
        return f'{word} ms'
    if address in (0x6027, 0x6028, 0x6029, 0x602A):
        return f'{number} rpm'
    if 0x602E <= address <= 0x6031:
        return f'{word}（驱动/ADC原始单位，非mA）'
    if address in (0x6007, 0x6012, 0x6013, 0x6018, 0x601A, 0x6024, 0x602B):
        return f'0x{word:04X}'
    if address == 0x7000:
        return f'{word}：' + {0: '命令清零（不是STOP）', 1: 'RUN', 5: '清故障', 8: 'STOP'}.get(word, '未知命令')
    if address == 0x7001:
        return f'{word}：' + {0: '参数命令字为空', 1: '请求保存', 2: '恢复默认并保存'}.get(word, '未知请求')
    return str(word)


def parse_exchange(tx_hex, rx_hex):
    """Validate a matched RTU exchange and describe its returned register words.

    CRC, node, function, length and write echo are mandatory. A correctly framed
    exception is reported as an exception, never as a successful register read.
    This parser has no stream buffering: provide one complete request/reply pair.
    """
    tx = _as_frame(tx_hex, '发送报文')
    if len(tx) != 8:
        raise ValueError('0x03/0x06请求必须为8字节')
    node, fc = tx[:2]
    address, value = int.from_bytes(tx[2:4], 'big'), int.from_bytes(tx[4:6], 'big')
    if request_frame(node, fc, address, value) != tx:
        raise ValueError('请求报文不符合0x03/0x06格式')
    rx = _as_frame(rx_hex, '接收报文')
    if rx[0] != node:
        raise ValueError('回包站号与请求不一致')
    if rx[1] == (fc | 0x80):
        if len(rx) != 5:
            raise ValueError('Modbus异常回包必须为5字节')
        reason = EXCEPTION_NAMES.get(rx[2], '未知异常码')
        return (f'ID{node} · 功能0x{fc:02X} · Modbus异常0x{rx[2]:02X}：{reason}',
                [(f'0x{address:04X}', 'Modbus异常（未确认执行成功）', f'{rx[2]:02X}', reason)])
    if rx[1] != fc:
        raise ValueError('回包功能码与请求不一致')
    if fc == 6:
        if rx != tx:
            raise ValueError('0x06回包必须完整回显请求；未确认写入')
        row = (f'0x{address:04X}', _label(address), _hex(tx[4:6]), _engineering(address, value))
        return f'ID{node} · 0x06写入应答已确认 · 0x{address:04X}（不等于掉电保存确认）', [row]
    if len(rx) != 5 + value * 2 or rx[2] != value * 2:
        raise ValueError('0x03回包字节数/长度与请求寄存器数不一致')
    rows = []
    for index in range(value):
        raw = rx[3 + index * 2:5 + index * 2]
        register, word = address + index, int.from_bytes(raw, 'big')
        rows.append((f'0x{register:04X}', _label(register), _hex(raw), _engineering(register, word)))
    return f'ID{node} · 0x03读取已确认 · 起址0x{address:04X} / {value}个寄存器', rows
