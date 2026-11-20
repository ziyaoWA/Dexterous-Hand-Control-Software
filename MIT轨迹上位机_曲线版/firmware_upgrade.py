"""Runtime RTU -> staged, pinned RAM kernel -> verified application upgrade.

The caller owns an already-open, exclusive port and the explicit operator
confirmation. Import and validate never open a port. Failures never retry an
erase/page or automatically reset. No EEPROM writing interface is used.
"""
from pathlib import Path
import binascii
import hashlib
import time

from updater_protocol import Updater, BAUD, crc16, load_hex

ROOT = Path(__file__).resolve().parent
APPLICATION_BAUD = 3000000
KERNEL = ROOT / 'ram_kernel_1110.bin'
KERNEL_SHA256 = 'fc47709d5cc2abe5aca92379c02693be333b1c970c22057a054378ccfd64f75e'
KERNEL_SIZE = 1492
KERNEL_CRC32 = 0x80D92961
PRODUCT = b'FJ485V1\0'


def _require(condition, message):
    if not condition:
        raise ValueError(message)


def _check_cancel(cancel):
    if cancel():
        raise InterruptedError('已取消；未自动复位。若已进入升级，请保持现场停机。')


def _check_node(node):
    _require(isinstance(node, int) and not isinstance(node, bool) and 1 <= node <= 5,
             '升级只允许整数单播站号1～5')


def _metadata(image):
    _require(image.project == 0x2508, 'HEX芯片型号不是MLX81339B02/0x2508')
    _require(image.data[0x2B0:0x2B8] == PRODUCT,
             'HEX不是FJ485V1控制固件；拒绝用不保留升级入口的旧程序覆盖')
    marker = int.from_bytes(image.data[0x2B8:0x2BA], 'little')
    _require(marker >= 0x1111, 'HEX应用版本低于0x1111，不是受支持的集成控制升级固件')
    legacy = marker == 0x1111
    warning = ('0x1111未包含升级软件复位后的RAM重建修复。写入校验通过后将留在RAM升级环境，'
               '不发送软件复位；请断电重上电，再读取应用版本和参数确认。') if legacy else ''
    return {'path': str(image.path), 'sha256': image.sha256, 'size': len(image.data),
            'project': image.project, 'image_crc32': image.crc, 'rom_bist': image.bist,
            'product': PRODUCT.rstrip(b'\0').decode('ascii'), 'normal_marker': marker,
            'requires_power_cycle': legacy, 'compatibility_warning': warning}


def validate(path):
    """Check the complete immutable HEX snapshot, chip, BIST and upgrade signature."""
    return _metadata(load_hex(path))


def _snapshot(path, expected_sha256):
    image = load_hex(path)
    details = _metadata(image)
    if expected_sha256 is not None:
        _require(image.sha256 == expected_sha256, 'HEX已变化，请重新校验并确认；未连接升级模式')
    kernel = KERNEL.read_bytes()
    _require(len(kernel) == KERNEL_SIZE and hashlib.sha256(kernel).hexdigest() == KERNEL_SHA256
             and binascii.crc32(kernel) == KERNEL_CRC32, '本地RAM内核文件损坏，禁止进入升级')
    return image, details, kernel


def _format(port, baud):
    changed = port.baudrate != baud
    if changed:
        # Drain the prior, acknowledged format before changing the UART.  Do
        # not touch DTR/RTS; IO3 on the target controls the 485 direction.
        port.flush()
    port.baudrate = baud
    port.bytesize = 8
    port.parity = 'N'
    port.stopbits = 1
    port.timeout = .03
    port.write_timeout = .5
    if changed:
        time.sleep(.03)
    port.reset_input_buffer()


def _rtu(port, node, function, argument, emit, count=None):
    _check_node(node)
    body = bytes((node, function)) + bytes(argument)
    frame = body + crc16(body).to_bytes(2, 'little')
    port.reset_input_buffer()
    port.write(frame)
    port.flush()
    emit('rtu_tx', frame.hex(' ').upper())
    reply = bytearray()
    deadline = time.monotonic() + .75
    while time.monotonic() < deadline:
        part = port.read(max(1, min(port.in_waiting, 128)))
        if part:
            reply.extend(part)
        if len(reply) >= 3:
            size = 5 if reply[1] & 0x80 else (reply[2] + 5 if function == 3 else 8)
            if len(reply) >= size:
                break
    emit('rtu_rx', bytes(reply).hex(' ').upper())
    if len(reply) < 5 or crc16(reply) != 0 or reply[0] != node:
        raise IOError('控制协议回包超时、CRC或站号不符；未发送升级BEGIN')
    if reply[1] == (function | 0x80):
        raise IOError(f'控制固件拒绝升级操作，Modbus异常码{reply[2]}；未发送升级BEGIN')
    _require(reply[1] == function, '控制回包功能码不符')
    if function == 3:
        _require(count is not None and len(reply) == 5 + 2 * count and reply[2] == count * 2,
                 '寄存器回包长度不符')
        return [int.from_bytes(reply[3+2*i:5+2*i], 'big') for i in range(count)]
    _require(len(reply) == 8 and reply[:6] == body, '控制写入/升级入口回包不是请求回显')
    return bytes(reply)


def _read(port, node, address, count, emit):
    return _rtu(port, node, 3, address.to_bytes(2, 'big') + count.to_bytes(2, 'big'), emit, count)


def read_info(port, node, emit=lambda *args: None, cancel=lambda: False):
    """Only 1-Mbps INFO. Does not request a runtime transition, write, or reset."""
    _check_node(node)
    _check_cancel(cancel)
    _require(port.baudrate == BAUD,
             '读取升级状态需要已打开的1Mbps升级连接；当前为控制连接时先用42进入升级态，或断开后仅开1Mbps升级串口。未发送任何字节')
    _format(port, BAUD)
    info = Updater(port, emit).read_info()
    _require(info['project'] == 0x2508 and info['marker'] == 0x1110 and info['write_capability'],
             '不是受支持的集成升级器')
    emit('upgrade_info', info)
    return info


def enter(port, node, emit=lambda *args: None, cancel=lambda: False):
    """Explicit STOP, verify idle, echo BOOT, then switch the same port to 1 Mbps."""
    _check_node(node)
    _check_cancel(cancel)
    _require(port.baudrate == APPLICATION_BAUD,
             '42入口需要已打开的3Mbps控制连接；已处于升级态请使用1Mbps升级连接继续。未发送任何字节')
    _format(port, APPLICATION_BAUD)
    marker = _read(port, node, 0x6024, 1, emit)[0]
    _require(marker >= 0x1111, '当前正常固件没有已支持的485升级入口；首次请使用原烧录器安装集成版')
    # Zero then STOP handles the edge-triggered H7000 command register.
    for command in (0, 8):
        _check_cancel(cancel)
        _rtu(port, node, 6, b'\x70\x00' + command.to_bytes(2, 'big'), emit)
    deadline = time.monotonic() + 2
    while True:
        _check_cancel(cancel)
        status = _read(port, node, 0x6006, 14, emit)
        speed = status[7] - 65536 if status[7] >= 32768 else status[7]
        calibration = status[3]
        if (status[0] == 0 and status[1] & 15 == 0 and abs(speed) <= 30
                and calibration not in (1, 2, 3, 4, 5, 6, 15)):
            break
        if time.monotonic() >= deadline:
            raise RuntimeError('电机尚未彻底停机/校准未结束，未发送升级入口')
        time.sleep(.05)
    _check_cancel(cancel)
    _rtu(port, node, 0x42, b'BOOT', emit)
    _format(port, BAUD)
    info = read_info(port, node, emit, cancel)
    _require(info['state'] == 6, '入口回应后未进入RAM内核装载阶段')
    return info


def _kernel_client(port, node, kernel, emit, cancel):
    _check_node(node)
    _check_cancel(cancel)
    selected_baud = port.baudrate
    _require(selected_baud in (APPLICATION_BAUD, BAUD),
             '连接波特率无效：正常控制请打开3Mbps，升级态请打开1Mbps；未发送任何字节')
    if selected_baud == APPLICATION_BAUD:
        # A live normal connection is already known to be 3 Mbps.  Sending an
        # unsolicited 1-Mbps sync/INFO first can disturb its UART receive state.
        emit('connection_route', {'mode': 'application', 'baudrate': selected_baud})
        info = enter(port, node, emit, cancel)
        client = Updater(port, emit)
    else:
        emit('connection_route', {'mode': 'upgrade', 'baudrate': selected_baud})
        _format(port, BAUD)
        client = Updater(port, emit)
        try:
            info = client.read_info()
        except TimeoutError as error:
            # This is a user-selected upgrade connection.  Do not guess a
            # different mode or send any RTU STOP/BOOT after a failed INFO.
            raise TimeoutError(
                '1Mbps升级INFO无回包，已停止，未切换到3Mbps猜测。'
                '若设备仍是正常应用，请断开后打开3Mbps控制连接；'
                '重新上电前须确认此前未开始擦写。本轮未发送擦写BEGIN、写页或复位。'
            ) from error
    _require(info['project'] == 0x2508 and info['marker'] == 0x1110 and info['write_capability'],
             '设备身份/实际写入权限不符')
    if info['state'] == 6:
        _require(0 <= info['next_page'] <= 12, '内核装载页序异常')
        # Each block ACK is checked. Resume only this fixed, pinned kernel.
        for block in range(info['next_page'], 12):
            _check_cancel(cancel)
            payload = kernel[block*128:(block+1)*128].ljust(128, b'\0')
            reply = client.exchange(0x30, block, payload)
            _require(reply['state'] == 6 and reply['next_page'] == block + 1,
                     '内核块确认序号不符，已停止')
            emit('kernel_progress', {'blocks': block + 1, 'total': 12})
        _check_cancel(cancel)
        loaded = client.exchange(0x31)
        _require(loaded['state'] == 6 and loaded['next_page'] == 12, '内核校验/移交确认失败')
        time.sleep(.03)
        info = client.read_info()
        _require(info['state'] == 0 and not info['checks_ready'], 'RAM内核未按预期接管')
    _require(info['state'] in (0, 3), '设备不在可开始升级状态；不自动重试擦除或恢复失败会话')
    return client


def dry_run(port, node, image_path, emit=lambda *args: None, cancel=lambda: False,
            expected_sha256=None):
    image, details, kernel = _snapshot(image_path, expected_sha256)
    client = _kernel_client(port, node, kernel, emit, cancel)
    def progress(done, total):
        emit('progress', {'stage': 'dry', 'pages': done, 'total': total})
    result = client.transfer(image, False, progress, cancel)
    emit('dry_verified', result)
    return {'dry_run_verified': True, 'physical_write': False, 'image': details, 'reply': result,
            'message': '完整传输演练通过，未实际写入或复位'}


def upgrade(port, node, image_path, emit=lambda *args: None, cancel=lambda: False,
            expected_sha256=None, allow_legacy=False):
    image, details, kernel = _snapshot(image_path, expected_sha256)
    if details['requires_power_cycle']:
        _require(allow_legacy is True,
                 '目标为0x1111旧版，需在上位机单独确认；写入后必须断电重上电，未开始升级')
    client = _kernel_client(port, node, kernel, emit, cancel)
    def progress(stage):
        return lambda done, total: emit('progress', {'stage': stage, 'pages': done, 'total': total})
    emit('stage', '完整传输演练')
    dry = client.transfer(image, False, progress('dry'), cancel)
    emit('dry_verified', dry)
    _check_cancel(cancel)
    emit('stage', '实际擦写应用区')
    written = client.transfer(image, True, progress('physical'), cancel)
    _require(written['observed_crc'] == image.crc and written['nv_crc'] == dry['nv_crc'],
             '写入校验不一致，禁止复位')
    emit('physical_verified', written)
    if details['requires_power_cycle']:
        # 1111 lacks the warm-reset noinit rebuild. Never run it with the RAM
        # state left by this updater: stay in RAM and require a real power cycle.
        result = {'verified': False, 'physical_write_verified': True,
                  'requires_power_cycle': True, 'image': details, 'physical_reply': written,
                  'nv_before': dry['nv_crc'], 'nv_after': written['nv_crc'],
                  'message': '旧版写入及CRC/NV校验通过；未发送软件复位。'
                             '请断电重上电，再连接3Mbps读取应用版本和参数验证。'}
        emit('legacy_write_verified', result)
        return result
    _check_cancel(cancel)
    reset = client.reset_verified()
    emit('reset_acknowledgement', reset)
    time.sleep(.25)
    _format(port, APPLICATION_BAUD)
    # A single fresh read, with no RUN and no speculative reset retries.
    application = _read(port, node, 0x6024, 1, emit)[0]
    _require(application == details['normal_marker'], '写入校验通过，但复位后新版本回读不符；请查看日志和当前站号')
    result = {'verified': True, 'image': details, 'physical_reply': written, 'reset_reply': reset,
              'application_marker': application, 'nv_before': dry['nv_crc'], 'nv_after': written['nv_crc'],
              'message': '升级成功：逐页/整包CRC和EEPROM校验通过，复位后新固件版本已确认'}
    emit('upgrade_verified', result)
    return result
