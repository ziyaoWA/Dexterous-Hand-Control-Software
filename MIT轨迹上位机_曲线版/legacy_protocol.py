"""Existing 0x68 / 0x69 / 0x72 transport, CRC16, no serial activity on import."""
import time
from tuning_core import crc16,signed

def control68(node,speed,position,kp=0,kd=0):
    if not 1<=node<=5 or not 1<=speed<=5000 or not -32768<=position<=32767:
        raise ValueError('0x68：站号1～5、速度1～5000、位置为有符号0.01°')
    data=bytearray((0,0x68))
    for index in range(1,node+1):
        words=(0x21,speed,position,0,kp,kd) if index==node else (0,0,0,0,0,0)
        for value in words:data.extend((value&65535).to_bytes(2,'big'))
    return bytes(data)+crc16(data)

def status_query(node,fc):
    if not 2<=node<=5 or fc not in (0x69,0x72):
        raise ValueError('链式查询针对站号2～5；站号1请用0x03读取M0')
    data=bytes((node-1,fc))+bytes(10)
    return data+crc16(data)

def decode_status(data):
    frames=[];offset=0
    while offset+14<=len(data):
        f=data[offset:offset+14]
        if 1<=f[0]<=5 and f[1]==0x72 and f[-2:]==crc16(f[:-2]):
            words=[int.from_bytes(f[i:i+2],'big') for i in range(2,12,2)]
            frames.append({'node':f[0],'stop_reason':words[0],'voltage_v':words[1]/100,
                'current_magnitude_ma':words[2],'speed_rpm':signed(words[3]),
                'position_deg_saturated':signed(words[4])/100})
            offset+=14
        else:offset+=1
    return frames

def exchange(session,frame):
    port=session.rtu.port
    port.reset_input_buffer();port.write(frame);port.flush()
    data=bytearray();deadline=time.monotonic()+.12
    while time.monotonic()<deadline:
        if session.rtu.interrupted():raise InterruptedError('操作者请求停止')
        data.extend(port.read(port.in_waiting or 1))
    frames=decode_status(data)
    session.log('legacy_wire',tx=frame.hex(' ').upper(),rx=data.hex(' ').upper(),decoded=frames)
    session.note('链式回包：'+str(frames) if frames else '已发送一次链式报文；未收到有效状态帧，详见日志。')
    return frames
