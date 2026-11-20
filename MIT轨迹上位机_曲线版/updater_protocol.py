"""Custom RAM-resident updater protocol; deliberately distinct from ROM UART.
No device is accessed on import. All destructive operations need explicit arming.
"""
from dataclasses import dataclass
from pathlib import Path
import binascii
import hashlib
import os
import struct
import subprocess
import tempfile
import time

FLASH_START=0x5800
IMAGE_SIZE=32768
PAGE_SIZE=128
BAUD=1000000
WRITE_ARM=0xC35A

def crc16(data):
    crc=0xFFFF
    for byte in data:
        crc^=byte
        for _ in range(8):crc=(crc>>1)^0xA001 if crc&1 else crc>>1
    return crc

@dataclass(frozen=True)
class Image:
    path:Path
    data:bytes
    project:int
    crc:int
    stream_crc:int
    sha256:str
    bist:int


def find_bist_tool(base=None):
    """Resolve the SDK checker in both standalone and full-project layouts."""
    root = Path(base).resolve() if base is not None else Path(__file__).resolve().parent
    candidates = [root / 'support/pymlxhex.exe']
    if len(root.parents) >= 2:
        candidates.append(root.parents[1] / 'code/camcu_platform/bin/pymlxhex.exe')
    for candidate in candidates:
        if candidate.is_file():
            return candidate
    locations = '\n'.join(str(candidate) for candidate in candidates)
    raise FileNotFoundError(
        '缺少ROM BIST校验工具pymlxhex.exe。请保留上位机support文件夹，'
        '或将上位机放回完整工程的tools/motor_console目录。\n'
        '已检查：\n' + locations + '\n未连接设备，未开始烧录。')


def load_hex(path,verify_bist=True):
    path=Path(path).resolve()
    if not path.is_file():
        raise FileNotFoundError(f'找不到目标HEX文件：{path}；请重新选择HEX，未开始烧录。')
    raw_hex=path.read_bytes()
    memory={};upper=0;eof=False
    for number,line in enumerate(raw_hex.decode('ascii').splitlines(),1):
        if not line.strip():continue
        if eof:raise ValueError(f'EOF之后还有记录：第{number}行')
        if not line.startswith(':'):raise ValueError('不是Intel HEX文件')
        record=bytes.fromhex(line[1:])
        if len(record)<5 or len(record)!=record[0]+5 or sum(record)%256:
            raise ValueError(f'HEX记录校验失败：第{number}行')
        count,address,kind=record[0],int.from_bytes(record[1:3],'big'),record[3]
        payload=record[4:-1]
        if kind==0:
            for i,value in enumerate(payload):
                at=upper+address+i
                if at<FLASH_START or at>=FLASH_START+IMAGE_SIZE:
                    raise ValueError('拒绝包含EEPROM/其他地址的HEX，只接受0x5800～0xD7FF应用区')
                if at in memory:raise ValueError('HEX地址重复')
                memory[at]=value
        elif kind==1 and count==0 and address==0:eof=True
        elif kind in (2,4) and count==2 and address==0:
            upper=int.from_bytes(payload,'big')<<(4 if kind==2 else 16)
        elif kind in (3,5) and count==4 and address==0:pass
        else:raise ValueError(f'不支持的HEX记录类型或长度：{kind}')
    if not eof or len(memory)!=IMAGE_SIZE:raise ValueError('HEX必须有EOF并完整覆盖32KiB应用区，拒绝自动填补缺失数据')
    data=bytes(memory[at] for at in range(FLASH_START,FLASH_START+IMAGE_SIZE))
    if int.from_bytes(data[0x26:0x28],'little')!=256:raise ValueError('只接受完整256页BIST布局')
    bist=int.from_bytes(data[-4:],'little')
    if verify_bist:
        tool=find_bist_tool()
        env=os.environ.copy()
        temp=Path(__file__).resolve().parent/'build_tmp'
        temp.mkdir(exist_ok=True)
        env['TEMP']=env['TMP']=str(temp)
        with tempfile.NamedTemporaryFile(prefix='bist_',suffix='.hex',dir=temp,delete=False) as frozen:
            frozen.write(raw_hex);snapshot=Path(frozen.name)
        assert snapshot.resolve().parent==temp.resolve()
        try:
            try:
                run=subprocess.run([str(tool),'calculate','--start=0x5800','--length=0x8000','rom_bist',str(snapshot)],
                    capture_output=True,text=True,env=env,timeout=30,creationflags=getattr(subprocess,'CREATE_NO_WINDOW',0))
            except OSError as error:
                raise RuntimeError(f'无法启动ROM BIST校验工具：{tool}；未开始烧录。原因：{error}') from error
            except subprocess.TimeoutExpired as error:
                raise RuntimeError(f'ROM BIST校验工具运行超时：{tool}；未开始烧录。') from error
        finally:snapshot.unlink(missing_ok=True)
        if run.returncode:
            diagnostic = (run.stderr or run.stdout or '').strip()[:500]
            raise ValueError(f'ROM BIST校验工具执行失败（退出码{run.returncode}）：{diagnostic}；禁止开始升级')
        try:
            observed_bist = int(run.stdout.strip(),0)
        except ValueError as error:
            raise ValueError('ROM BIST校验工具输出无效，禁止开始升级') from error
        if observed_bist!=bist:raise ValueError('ROM BIST校验不通过，禁止开始升级')
    return Image(path,data,int.from_bytes(data[6:8],'little')^0xA55A,binascii.crc32(data),
        binascii.crc32(data[128:]+data[:128]),hashlib.sha256(raw_hex).hexdigest(),bist)

def make_request(command,sequence,argument=0,payload=b''):
    if len(payload)>128:raise ValueError('Payload exceeds one page')
    frame=bytearray(144)
    struct.pack_into('<2sBBH',frame,0,b'\x55\xAA',command,sequence,argument)
    frame[6:6+len(payload)]=payload
    struct.pack_into('<H',frame,142,crc16(frame[:142]))
    return bytes(frame)

def parse_response(frame):
    if len(frame)!=24 or frame[:2]!=b'\x55\xAB' or crc16(frame):raise ValueError('回包头、长度或CRC错误')
    next_page,marker,project=struct.unpack_from('<HHH',frame,4)
    ram_start,ram_end,observed_crc,nv_crc=struct.unpack_from('<HHIH',frame,12)
    if marker not in (0x110F,0x1110) or not 0x1000<=ram_start<ram_end<0x17E2:
        raise ValueError('不是受支持的RAM升级器，或RAM范围异常')
    if bool(frame[10]&0x80)!=(marker==0x1110):raise ValueError('升级器版本与写入权限不一致')
    return {'command':frame[2],'status':frame[3],'next_page':next_page,'marker':marker,'project':project,
            'state':frame[10]&0x0F,'checks_ready':bool(frame[10]&0x40),'write_capability':bool(frame[10]&0x80),'sequence':frame[11],
            'ram_start':ram_start,'ram_end':ram_end,'observed_crc':observed_crc,'nv_crc':nv_crc}

class Updater:
    def __init__(self,port,emit=lambda *args:None):
        self.port=port;self.emit=emit;self.sequence=0;self.identity=None
        self.last_dry=None;self.last_write=None
    def exchange(self,command,argument=0,payload=b'',timeout=2):
        self.sequence=(self.sequence+1)&255
        request=make_request(command,self.sequence,argument,payload)
        self.port.reset_input_buffer();self.port.write(request);self.port.flush()
        self.emit('tx',request.hex(' ').upper())
        buffer=bytearray();deadline=time.monotonic()+timeout
        while time.monotonic()<deadline:
            data=self.port.read(max(1,min(self.port.in_waiting,128)))
            if data:self.emit('rx',data.hex(' ').upper());buffer.extend(data)
            while len(buffer)>=24:
                if buffer[:2]!=b'\x55\xAB':del buffer[0];continue
                candidate=bytes(buffer[:24])
                try:response=parse_response(candidate)
                except ValueError:del buffer[0];continue
                del buffer[:24]
                if response['sequence']!=self.sequence or response['command']!=command:continue
                if response['status']:raise RuntimeError(f"设备拒绝命令0x{command:02X}，错误码{response['status']}；本轮不自动重发")
                time.sleep(.002)
                return response
        raise TimeoutError('读取超时，已停止本轮；不自动重发、不自动复位。')
    def read_info(self):
        self.port.write(bytes(288));self.port.flush();time.sleep(.005) # frame-boundary recovery only; no valid command
        self.identity=self.exchange(1)
        return self.identity
    def transfer(self,image,physical=False,progress=lambda *args:None,cancel=lambda:False):
        info=self.exchange(1)
        if info['project']!=image.project:raise ValueError('HEX与芯片型号编码不匹配，未发送BEGIN')
        if physical and not info['write_capability']:raise ValueError('当前固件禁止实际擦写；只能传输演练')
        if not info['checks_ready']:
            self.emit('preparing_checks','建立Flash和EEPROM只读校验基线')
            info=self.exchange(2,timeout=5)
            if not info['checks_ready']:raise ValueError('设备尚未建立校验基线，未发送BEGIN')
        fingerprint=tuple(info[key] for key in ('marker','project','observed_crc','nv_crc'))
        if physical and self.last_dry!=(image.sha256,fingerprint):
            raise ValueError('需要在同一连接中先对同一份HEX完成演练，且设备和校验基线保持一致；未发送BEGIN')
        self.last_dry=None
        manifest=struct.pack('<IIIHHH',0x55475031,image.crc,image.stream_crc,image.project,IMAGE_SIZE,WRITE_ARM if physical else 0)
        if cancel():raise RuntimeError('开始前已取消')
        result=self.exchange(0x20,payload=manifest,timeout=12)
        if result['state']!=(2 if physical else 1):raise ValueError('设备未进入预期模式')
        for done,page in enumerate(list(range(1,256))+[0],1):
            if cancel():raise RuntimeError('用户取消：未发送复位；若为实际升级，设备仍处于RAM升级环境')
            result=self.exchange(0x21,page,image.data[page*128:(page+1)*128])
            expected=0 if page==255 else (256 if page==0 else page+1)
            if result['next_page']!=expected:raise ValueError('设备确认页序不一致，本轮停止')
            progress(done,256)
        result=self.exchange(0x22,timeout=5)
        if result['state']!=(4 if physical else 3):raise ValueError('完整校验未通过')
        if result['nv_crc']!=info['nv_crc']:raise ValueError('EEPROM校验发生变化')
        if physical:
            if result['observed_crc']!=image.crc:raise ValueError('Flash整包读回CRC与目标不符')
        elif result['observed_crc']!=info['observed_crc']:raise ValueError('演练前后Flash基线CRC不一致')
        if physical:self.last_write=(image.sha256,result)
        else:self.last_dry=(image.sha256,fingerprint)
        return result
    def reset_verified(self):
        if self.last_write is None:raise ValueError('本连接没有完成校验的实际写入，不发送复位')
        return self.exchange(0x23,0x5253)
