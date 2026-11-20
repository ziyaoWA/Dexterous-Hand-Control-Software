"""Complete RTU and legacy-chain services for the dedicated 0x1108 firmware.
All writes are explicit UI jobs. Manual RUN and calibration are time bounded.
"""
import json
import time
import uuid
from datetime import datetime, timezone
from tuning_core import Session, crc16, signed, response_metrics, supports_mit, supports_position_hold
from legacy_protocol import control68,status_query,exchange

CALIBRATION_TIMEOUT_SECONDS = 300

F0_NAMES = ['控制模式','速度/速度上限 rpm','绝对位置 0.01°','电流给定 mA',
            'MIT Kp Q15','MIT Kd Q15','速度Kp Q12','速度Ki Q12/周期',
            '位置Kp Q15','电流Kp Q12','电流Ki Q12','站号 1～5',
            '全局速度上限 rpm','峰值限流 mA（0禁止驱动）','ESO b0×10',
            'ESO带宽 rad/s','ESO补偿 Q10','ESO输出 mA（只读）',
            '位置Ki Q25/周期','位置Kd Q12']
F1_NAMES = ['额定/过流保护基准 mA','一级过流确认 ms','二级过流确认 ms','三级过流确认 ms',
            '通讯超时 ms','位置跳变 counts','堵转低速 rpm','堵转电流 ×100',
            '欠压 10mV','过压 10mV','过温 ℃','过压电流比例 ×100','故障确认 ms',
            '位置到位死区 0.01°（1114）','位置保持峰值上限 mA（1114）','退出保持额外滞回 0.01°（1114）']
M0_NAMES = ['末次命令','控制模式','生效速度给定','生效位置给定','速度请求 rpm','电流请求 mA',
            '驱动请求','驱动状态','命令计数','校准步骤','停止原因','母线电压 10mV',
            '实际峰值电流 mA','实际转速 rpm','位置 0.01°（饱和）','Resolver X','Resolver Y',
            '目标位置 0.01°','电气错误','电压错误','故障码','故障等级','通讯空闲 ms',
            '芯片温度 ℃','故障详情','过流降额 Q12','位置状态位','绝对累计计数高16',
            '绝对累计计数低16','驱动累计计数高16','驱动累计计数低16','驱动目标高16',
            '驱动目标低16','驱动剩余距离','微步索引','驱动停止原因','固件标识（1113/1114支持MIT）',
            '位置误差 counts 高16','位置误差 counts 低16','位置P输出 rpm','位置I输出 rpm',
            '位置D输出 rpm','位置PID限幅前 rpm','位置PID状态位（0x0100到位保持）','速度积分 mA',
            '当前命令域电流上限 mA（降额前）','110A: coPRO目标Ipk ADC-LSB','110A: 电流PI输出 原始值',
            '110A/B: coPRO电压修正 原始值','110A主核PWM幅值 / 110B PWM上限']
H_NAMES = ['命令：0空/1运行/5清故障/8停止','参数：1保存/2恢复默认并保存',
           '保留2','保留3','保留4','保留5','保留6','保留7','保留8','保留9']
GROUPS = {'F0':(0,F0_NAMES),'F1':(0x100,F1_NAMES),'M0':(0x6000,M0_NAMES),'H':(0x7000,H_NAMES)}


def register_span(address,count):
    if not 1 <= count <= 20:
        raise ValueError('单包读取1～20个寄存器')
    if not any(base <= address and address+count <= base+len(names) for base,names in GROUPS.values()):
        raise ValueError('读取范围必须完整位于F0、F1、M0或H组内')


def validate_word(address,value,marker=None):
    if not -32768 <= value <= 65535:
        raise ValueError('输入须为16位整数，可填写负数或0～65535原始字')
    word=value & 65535
    number=signed(word)
    if 0 <= address < 20:
        if address == 17:
            raise ValueError('F0-17是只读ESO输出')
        if address == 0 and word not in (0,1,2,100) and not (word == 3 and supports_mit(marker)):
            raise ValueError('模式支持电流0、速度1、位置2、校准100；MIT模式3仅限已识别的0x1113或0x1114固件')
        if address == 1 and abs(number)>5000:
            raise ValueError('速度给定范围±5000rpm')
        if address == 3 and abs(number)>1212:
            raise ValueError('电流给定范围±1212mA')
        if address == 11 and not 1<=word<=5:
            raise ValueError('站号范围1～5')
        if address == 12 and not 0<=word<=5000:
            raise ValueError('全局速度上限0～5000，0采用固件默认')
        if address == 13 and word>1212:
            raise ValueError('峰值限流0～1212mA，0禁止驱动')
        if address == 16 and word>1024:
            raise ValueError('ESO补偿范围0～1024')
        if address not in (0,1,2,3,11,12,13,16) and word>32767:
            raise ValueError('该参数使用非负有符号16位整数')
    elif 0x100<=address<0x110:
        if address >= 0x10D:
            if not supports_position_hold(marker):
                raise ValueError('F1-13～15位置保持参数仅限已识别的0x1114固件；旧版是保留字')
            maximum = 1212 if address == 0x10E else 100
            if word > maximum:
                raise ValueError('保持峰值上限0～1212mA，0跟随F0-13' if address == 0x10E
                                 else '死区/额外滞回原始值0～100，单位0.01°；0采用默认5')
    elif 0x7000<=address<0x700A:
        if address==0x7000 and word not in (0,1,5,8):
            raise ValueError('H7000命令仅0、1、5、8')
        if address==0x7001 and word not in (0,1,2):
            raise ValueError('H7001：1保存，2恢复默认并保存，0无操作')
    else:
        raise ValueError('M0只读；地址超出已支持的可写寄存器')
    return word


class ProtocolSession(Session):
    def __init__(self,*args,**kwargs):
        super().__init__(*args,**kwargs)
        self.service_motion=None

    def read_registers(self,address,count):
        register_span(address,count)
        body=bytes((self.rtu.node,3))+address.to_bytes(2,'big')+count.to_bytes(2,'big')
        return self.rtu.transact(body,count)

    def write_register(self,address,value,fc=6):
        marker=(self.identity or {}).get('marker') if self.verified else None
        word=validate_word(address,value,marker)
        if fc not in (6,0x41) or fc==0x41 and not 0x7000<=address<0x700A:
            raise ValueError('0x41仅用于H7000组，普通寄存器请用0x06')
        body=bytes((self.rtu.node,fc))+address.to_bytes(2,'big')+word.to_bytes(2,'big')
        return self.rtu.transact(body)

    def protocol(self,payload):
        if not self.verified:
            raise RuntimeError('先点②读取参数，核对配套控制固件')
        action=payload['action']
        if action=='legacy_query':
            self.assert_idle()
            exchange(self,status_query(self.rtu.node,int(payload['function'])))
            return
        if action=='legacy68':
            self.assert_idle()
            speed=int(payload['speed']);position=int(payload['position'])
            control68(self.rtu.node,speed,position) # validate before any write
            self.rtu.stop();self.require_stopped()
            self.write_register(0,2);self.write_register(1,speed);self.write_register(2,position)
            self.start_service({**payload,'legacy68':True})
            return
        address=int(payload.get('address',0))
        if action=='read':
            count=int(payload.get('count',1))
            values=self.read_registers(address,count)
            self.log('protocol_read',address=address,values=values)
            self.send('protocol_result',{'address':address,'values':values})
            return
        value=int(payload.get('value',0));fc=int(payload.get('fc',6))
        validate_word(address,value,(self.identity or {}).get('marker'))
        if address==0x7000 and value==8:
            if fc==0x41:
                self.write_register(0x7000,0,fc)
                self.write_register(0x7000,8,fc)
            self.stop('完整协议页：停止')
            return
        if address==0x7000 and value==1:
            self.start_service(payload)
            return
        self.assert_idle()
        if address==0x7001 and value==1:
            self.save({'fc':fc})
            return
        if address==0x7001 and value==2:
            self.restore_defaults(fc)
            return
        self.rtu.stop();self.require_stopped()
        if address==0x7000 and value==5:
            self.write_register(0x7000,0,fc)
        self.write_register(address,value,fc)
        if address==11:
            self.rtu.node=value
            self.connection_info['node']=value
            self.verified=False
            self.send('node_changed',value)
        if address<20 or 0x100<=address<0x110:
            self.read_parameters()
            actual=self.f0[address] if address<20 else self.f1[address-0x100]
            if actual!=(value&65535):raise IOError('写入读回不一致')
            self.send('protocol_result',{'address':address,'values':[actual]})
        else:
            self.read_status()
        self.note(f'已执行0x{fc:02X}写0x{address:04X}={value}；尚未作为掉电保存成功。')

    def save(self,payload=None):
        fc=int((payload or {}).get('fc',6))
        if fc==6:
            return super().save()
        self.assert_idle();self.rtu.stop();self.require_stopped();self.read_parameters()
        if self.f0[0] not in (0,1,2) and not (self.f0[0]==3 and supports_mit((self.identity or {}).get('marker'))):
            raise ValueError('保存前退出校准或不受当前固件支持的模式；MIT需要0x1113或0x1114')
        expected={**self.identity,'f0':self.f0.copy(),'f1':self.f1.copy(),
                  'time_utc':datetime.now(timezone.utc).isoformat()}
        self.write_register(0x7001,1,fc)
        self.pending_path.write_text(json.dumps(expected,ensure_ascii=False,indent=2),encoding='utf-8')
        self.log('save_requested_not_yet_verified',expected=expected)
        self.send('save_state','已发送0x41保存请求，待断电重上电核验')

    def restore_defaults(self,fc):
        self.rtu.stop();self.require_stopped()
        self.write_register(0x7001,2,fc)
        time.sleep(.2)
        self.read_parameters()
        expected={**self.identity,'f0':self.f0.copy(),'f1':self.f1.copy(),
                  'time_utc':datetime.now(timezone.utc).isoformat(),'operation':'restore_defaults'}
        self.pending_path.write_text(json.dumps(expected,ensure_ascii=False,indent=2),encoding='utf-8')
        self.send('save_state','恢复默认及保存请求已发出，待重上电核验')
        self.note('已读回默认参数；站号由固件保留。存储结果仍须重上电核验。')

    def start_calibration(self,payload=None):
        """Explicit UI action: STOP, select factory calibration, then one RUN."""
        self.assert_idle()
        self.read_parameters()
        state=self.require_stopped()
        if any(state[k] for k in ('fault','electric_error','voltage_error')):
            raise ValueError('先排查实际故障，校准不会自动清故障')
        if not 1<=self.f0[13]<=1212:
            raise ValueError('校准峰值限流须为1～1212mA；请先应用限幅并读回')
        self.exit_plan()
        self.protocol({'action':'write','address':0,'value':100,'fc':6})
        self.start_service({'required_mode':100,'fc':6,'seconds':2,
                            'speed_guard':6000,'travel_guard':360})

    def start_service(self,payload):
        self.assert_idle();self.read_parameters()
        state=self.require_stopped()
        if state['fault'] or state['electric_error'] or state['voltage_error']:
            raise ValueError('先排查并清除实际故障，再启动')
        mode=self.f0[0]
        if 'required_mode' in payload and mode!=payload['required_mode']:
            raise ValueError('校准模式读回不一致，未发送RUN')
        if mode not in (0,1,2,100) and not (mode==3 and supports_mit((self.identity or {}).get('marker'))):
            raise ValueError('当前固件不支持所选模式；MIT限时RUN需要0x1113或0x1114')
        if not 1<=self.f0[13]<=1212:
            raise ValueError('模式无效或电流限幅为0，拒绝启动')
        seconds=float(payload.get('seconds',2))
        speed_guard=float(payload.get('speed_guard',1000))
        travel_guard=float(payload.get('travel_guard',90))
        if not .2<=seconds<=10 or not 100<=speed_guard<=6000 or not .5<=travel_guard<=360:
            raise ValueError('手动测试限时0.2～10秒，转速停止阈值100～6000，偏移0.5～360°')
        if mode!=100 and state['calibration_step']!=10:
            raise ValueError('先完成校准')
        if mode!=100 and (state['position_flags'] & 0x37)!=1:
            raise ValueError('位置反馈未就绪')
        self.rtu.stop();self.require_stopped()
        target=(signed(self.f0[3]),signed(self.f0[1]),signed(self.f0[2])/100,signed(self.f0[2])/100)[mode] if mode!=100 else 0
        identifier=('calibration_' if mode==100 else 'manual_')+uuid.uuid4().hex[:8]
        self.service_motion={'id':identifier,'mode':mode,'started':self.clock(),'baseline':state,
            'seconds':CALIBRATION_TIMEOUT_SECONDS if mode==100 else seconds,'speed_guard':6000 if mode==100 else speed_guard,
            'travel_guard':travel_guard,'f0':self.f0.copy(),'f1':self.f1.copy(),
            'current_limit_ma':self.f0[13],'samples':[],'seen_calibration_active':False,
            'target':target,'restore_mode':1 if mode==100 else mode}
        self.send('service_state',{'active':True,'id':identifier,'calibration':mode==100})
        if payload.get('legacy68'):
            exchange(self,control68(self.rtu.node,signed(self.f0[1]),signed(self.f0[2]),self.f0[4],self.f0[5]))
        else:
            self.write_register(0x7000,0,int(payload.get('fc',6)))
            self.write_register(0x7000,1,int(payload.get('fc',6)))
        self.log('operator_started_service',service={k:v for k,v in self.service_motion.items() if k!='samples'})
        self.note('已发送一次RUN；'+(f'校准最长{CALIBRATION_TIMEOUT_SECONDS}秒，独立监视校准步骤。' if mode==100 else f'按当前F0给定运行，最多{seconds}秒。'))

    def poll_service(self):
        service=self.service_motion
        if not service:return
        state=self.read_status()
        elapsed=self.clock()-service['started']
        service['samples'].append({**state,'elapsed':elapsed,'current_limit_ma':service['current_limit_ma']})
        if service['mode']==100:
            diagnostic=None
            if self.identity.get('marker') in (0x110A,0x110B,0x1111,0x1112,0x1113,0x1114):
                words=self.read_registers(0x602E,4)
                last_key='pwm_limit_raw' if self.identity['marker'] in (0x110B,0x1111,0x1112,0x1113,0x1114) else 'main_pwm_raw'
                diagnostic=dict(zip(('target_ipk_lsb','current_pi_raw','copro_voltage_raw',last_key),words))
                service['samples'][-1].update(diagnostic)
                self.log('calibration_drive_diag',service_id=service['id'],step=state['calibration_step'],**diagnostic)
            self.send('calibration_progress',{'step':state['calibration_step'],'elapsed':elapsed,
                      'request':state['motor_request'],'status':state['motor_status'],'drive_diag':diagnostic})
        reason=None
        if state['fault'] or state['electric_error'] or state['voltage_error']:
            reason=(f"{'校准' if service['mode']==100 else '服务测试'}停止："
                    f"步骤{state['calibration_step']}，故障码{state['fault']}，"
                    f"电气错误{state['electric_error']}，电压错误{state['voltage_error']}")
        elif abs(state['speed_actual_rpm'])>service['speed_guard']:reason='服务测试超过转速停止阈值'
        elif state['current_feedback_magnitude_ma']>service['current_limit_ma']*1.5:reason='服务测试电流异常'
        elif service['mode']==100:
            step=state['calibration_step']
            if state['motor_request']==8 or 1<=step<=6:service['seen_calibration_active']=True
            if 11<=step<=14:reason='校准失败步骤'+str(step)
            elif step==10 and service['seen_calibration_active']:reason='校准步骤10，待重上电核验标定存储'
        elif (state['position_flags'] & 0x37)!=1:reason='服务测试位置反馈失效'
        elif abs(state['position_actual_deg']-service['baseline']['position_actual_deg'])>service['travel_guard']:reason='服务测试超过最大位置偏移'
        elif state['mode']!=service['mode']:reason='服务测试控制模式被改变'
        if reason is None and elapsed>=service['seconds']:
            reason=f"校准超时{service['seconds']}秒，未确认完成" if service['mode']==100 else '服务测试时长已到'
        if reason:self.stop(reason)

    def stop(self,reason='操作者点击立即停止'):
        service=self.service_motion
        ok=False
        try:
            super().stop(reason)
            ok=True
            if service and service['mode']==100:
                self.write_register(0,service['restore_mode'])
                self.read_parameters()
        finally:
            if service:
                self.service_motion=None
                service.update(stop_reason=reason,stop_state_confirmed=ok)
                path=self.run_dir/(service['id']+'.json')
                path.write_text(json.dumps(service,ensure_ascii=False,indent=2),encoding='utf-8')
                self.log('service_finished',id=service['id'],reason=reason,stopped=ok)
                self.send('service_state',{'active':False,'id':service['id'],'reason':reason,'stopped':ok,
                          'calibration':service['mode']==100})
                if service['mode']!=100:
                    mode=service['mode'];key=('current_feedback_magnitude_ma','speed_actual_rpm','position_actual_deg')[2 if mode==3 else mode]
                    trial={**service,'path':str(path),'spec':{'mode':mode,'current_limit_ma':service['current_limit_ma']},
                        'metrics':response_metrics(service['samples'],mode,service['target'],service['baseline'][key])}
                    self.send('trial_finished',trial)
