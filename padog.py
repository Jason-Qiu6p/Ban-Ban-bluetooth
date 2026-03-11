#加载预置参数
try:
    with open('config_s.py') as f:
        exec(f.read())
except:
    pass

#引入模块
import PA_SERVO
import PA_GAIT
import PA_IK
import PA_ATTITUDE
import PA_STABLIZE
import time
import ujson
from math import sin,cos,pi,atan,tan,floor
from machine import Pin,PWM,ADC,Timer,time_pulse_us,UART

# ====== 调试开关 ======
DEBUG_USB_MODE = True  # 如果为 True，即使检测到电压过低(USB供电)也不切断舵机，方便调试

# 适配新版 MicroPython 的 PWM 和 ADC
adc = ADC(Pin(33))
adc.atten(ADC.ATTN_11DB)
adc.width(ADC.WIDTH_12BIT) 

pin_servo_vol = Pin(25,Pin.OUT)

#======UART=======
# 语音模块串口
uart6=UART(2,115200,tx=26,rx=27)
uart_per_add=0

# ================= 协议解析层 =================

def process_joystick(x, y):
    """处理摇杆数据 (非阻塞)"""
    global spd_goal, L, R, current_pose, timed_action_running
    
    if current_pose != 'stand':
        stand_pose() 
        
    timed_action_running = False 

    if abs(x) < 10 and abs(y) < 10:
        move(0, 0, 0)
        return

    target_spd = (y / 100.0) * 3.0
    if target_spd > 3: target_spd = 3
    if target_spd < -3: target_spd = -3
    
    target_L = 1
    target_R = 1
    
    if x >= 40:  # 右转
        target_L = 1; target_R = -1
        if abs(target_spd) < 0.5: target_spd = 1.5 
    elif x <= -40: # 左转
        target_L = -1; target_R = 1
        if abs(target_spd) < 0.5: target_spd = 1.5

    move(target_spd, target_L, target_R)

def parse_command(raw_data, source="UART"):
    """
    统一指令解析器: 支持 JSON (蓝牙/串口) 和 HEX (语音/串口)
    """
    global CC_M, timed_action_running
    
    str_data = ""
    try:
        if isinstance(raw_data, bytes):
            str_data = raw_data.decode('utf-8').strip()
        else:
            str_data = raw_data.strip()
    except:
        pass

    # 2. 尝试解析 JSON
    if str_data.startswith('{'):
        json_list = str_data.replace('}{', '}|{').split('|')
        
        for j_str in json_list:
            try:
                json_obj = ujson.loads(j_str)

                if "x" in json_obj and "y" in json_obj:
                    process_joystick(int(json_obj["x"]), int(json_obj["y"]))
                    # 注意：这里不 return，因为可能粘包的后面还有指令，需全部执行完

                if "DJCtrl" in json_obj:
                    handle_action(json_obj["DJCtrl"])
            except:
                pass 
        return # 如果是 JSON 格式，处理完直接返回

    # 3. 尝试解析 HEX (语音模块)
    if isinstance(raw_data, bytes):
        if raw_data == b'\xFF\x01\xEF': forward_3s_action()
        elif raw_data == b'\xFF\x02\xEF': backward_3s_action()
        elif raw_data == b'\xFF\x03\xEF': left_3s_action()
        elif raw_data == b'\xFF\x04\xEF': right_3s_action()
        elif raw_data == b'\xFF\x05\xEF': 
            move(0,0,0)
            timed_action_running = False
        elif raw_data == b'\xFF\x06\xEF': stand_pose()
        elif raw_data == b'\xFF\x07\xEF': sit_pose()
        elif raw_data == b'\xFF\x08\xEF': lie_pose()
        elif raw_data == b'\xFF\x11\xEF': stretch()
        elif raw_data == b'\xFF\x12\xEF': shake_body()
        elif raw_data == b'\xFF\x13\xEF': dance()
        elif raw_data == b'\xFF\x14\xEF': push_up()

def handle_action(action):
    global timed_action_running
    if action == "stop":
        move(0, 0, 0)
        timed_action_running = False
    elif action == "stand": stand_pose()
    elif action == "sit": sit_pose()
    elif action == "lie": lie_pose()
    elif action == "dance": dance()
    elif action == "stretch": stretch()
    elif action == "push_up": push_up()
    elif action == "shake_body": shake_body()
    elif action == "greet": greet_action()
    elif action == "gait_trot": gait(0)
    elif action == "gait_walk": gait(1)
    elif action == "stable_on": stable(True)
    elif action == "stable_off": stable(False)
    elif action == "forward": forward_3s_action()
    elif action == "backward": backward_3s_action()
    elif action == "left": left_3s_action()
    elif action == "right": right_3s_action()

def UART_Run():
    global uart_per_add
    uart_per_add = uart_per_add + 5
    if uart_per_add >= 50:
        uart_per_add = 0
        if uart6.any():
            raw_data = uart6.read(uart6.any())
            parse_command(raw_data, source="UART")
            
#======REMOTER======
def fb_curve(x):
  p1 = 5.334e-06; p2 = -0.02423; p3 = 23.92
  return p1*x*x + p2*x + p3
  
remote_per_add=0
micros1_last=0;micros2_last=0
micros4_count_1=0;micros4_count_2=0;micros4_node_1=0;micros4_node_2=0
micros6_count_1=0;micros6_count_2=0;micros6_node_1=0;micros6_node_2=0

def remote_run():
  global remote_per_add,micros1_last,micros2_last
  global micros4_count_1,micros4_count_2,micros4_node_1,micros4_node_2
  global micros6_count_1,micros6_count_2,micros6_node_1,micros6_node_2
  
  remote_per_add=remote_per_add+5

  if remote_per_add>=90:
    remote_per_add=0
    try:
        micros1 = time_pulse_us(Pin(32,Pin.IN), 1, 20000)
        micros2 = time_pulse_us(Pin(26,Pin.IN), 1, 20000)
        micros6 = time_pulse_us(Pin(27,Pin.IN), 1, 20000)
        micros4 = time_pulse_us(Pin(14,Pin.IN), 1, 20000)
    except: return

    if micros1 < 0 or micros2 < 0: return

    if micros4>1500: micros4_count_2=0; micros4_count_1+=1
    else: micros4_count_1=0; micros4_count_2+=1
    
    if micros4_count_1>=3: micros4_node_1=1; micros4_node_2=0; micros4_count_1=0
    if micros4_count_2>=3: micros4_node_1=0; micros4_node_2=1; micros4_count_1=0

    if micros6>1500: micros6_count_2=0; micros6_count_1+=1
    else: micros6_count_1=0; micros6_count_2+=1
    
    if micros6_count_1>=3: micros6_node_1=1; micros6_node_2=0; micros6_count_1=0
    if micros6_count_2>=3: micros6_node_1=0; micros6_node_2=1; micros6_count_1=0 
    
    if micros6_node_1==1 and (micros1+micros2+micros6+micros4)>1000: stable(True)
    else: stable(False)

    val_curve = floor(fb_curve(micros2))
    if val_curve>=6: thr=6
    elif val_curve<=-3: thr=-3
    else: thr=val_curve

    if micros4_node_1==1 and (micros1+micros2+micros6+micros4)>1000:
      if micros1<=(1300) and abs(micros1_last-micros1)<100: move(2.5,-1,1)
      elif micros1>=(1700) and abs(micros2_last-micros2)<100: move(2.5,1,-1)
      else:
        if abs(micros2_last-micros2)<50: move(thr,1,1)
    else:
      move(0,0,0)
    
    micros1_last=micros1; micros2_last=micros2

#======REMOTER======

#======警告======
alarm_pin = PWM(Pin(13), freq=1000, duty=0)

alarm_flash_node=0
sound_freq1=0; sound_freq2=0
alarm_per=0; alarm_per_add=0; alarm_time_per=0
loop_speed_mode=0; loop_speed_mode_sc=0

def alarm_run():
  global alarm_flash_node,alarm_per_add
  alarm_per_add=alarm_per_add+5
  if alarm_per_add>=alarm_time_per:
    alarm_per_add=0
    if alarm_flash_node==0:
      if sound_freq1 > 0: alarm_pin.freq(sound_freq1)
      alarm_flash_node=1
    else:
      alarm_flash_node=0
      if sound_freq2 > 0: alarm_pin.freq(sound_freq2)

def alarm(al_per,s_freq,s_freq2):
  global sound_freq1,sound_freq2,alarm_time_per
  sound_freq1=s_freq; sound_freq2=s_freq2
  if al_per==0:
    alarm_pin.duty(0)
    alarm_time_per=0
  else:
    alarm_time_per=al_per
    if s_freq > 0: alarm_pin.freq(s_freq)
    alarm_pin.duty(512)
    
def start_ring():
  alarm(10,100,100); time.sleep(0.15)
  alarm(10,300,300); time.sleep(0.15)
  alarm(10,400,400); time.sleep(0.15)
  alarm(10,600,600); time.sleep(0.15)
  alarm(10,800,800); time.sleep(0.15)
  alarm(0,0,0); time.sleep(1)
#======警告======

#连接网络
selfadd=0
def do_connect_AP():
  global selfadd
  import network 
  wifi = network.WLAN(network.AP_IF) 
  wifi.config(essid='Ban-Ban Dog - 伴伴智宠V3')
  if not wifi.isconnected(): 
    print('热点 WIFI 已开启，WIFI名称：Ban-Ban Dog - 伴伴智宠')
    print('等待手机接入...')
    alarm(300,460,0)
    wifi.active(True) 
    while not wifi.isconnected():
      pass
  selfadd=wifi.ifconfig()[0]
  print("手机热点成功接入")
  alarm(10,660,460); time.sleep(0.5); alarm(0,0,0)

#=============一些中间或全局变量=============
t=0
init_x=0;init_y=-110
ges_x_1=0;ges_x_2=0;ges_x_3=0;ges_x_4=0
ges_y_1=init_y;ges_y_2=init_y;ges_y_3=init_y;ges_y_4=init_y
x1=0;x2=0;x3=0;x4=0;y1=0;y2=0;y3=0;y4=0;
PIT_S=0;ROL_S=0;X_S=0;PIT_goal=0;ROL_goal=0;X_goal=0
spd=0;spd_goal=0;L=0;R=0
R_H=abs(init_y);H_goal=110
init_case=0
key_stab=False;gait_mode=0
stop_run_node=0
speed_init=speed
acc_z=0
IK_ERROR=0;normal_node=0;error_node=0;empty_power_count=0
pit_max_ang=25; rol_max_ang=20
Kp_V=0.2
act_tran_mov_kp=tran_mov_kp
timed_action_end_time = 0
timed_action_running = False
current_pose = 'stand'
last_servo_positions = None  # 追踪最后一次舵机输出的8个角度，用于平滑过渡
has_printed_initial_voltage = False  # 新增：用于确保电压仅在上电时显示一次

def mechan_offset_corr(x):
  p1=0.006649; p2=0.4414; p3=5.53
  return p1*x*x+p2*x+p3

def read_voltage(x):
    return 0.01235 * x

def servo_output(case,init,ham1,ham2,ham3,ham4,shank1,shank2,shank3,shank4):
  global last_servo_positions
  if case==0 and init==0:
    PA_SERVO.angle(2, init_1h+90-ham1)
    PA_SERVO.angle(3, (init_1s-90)+mechan_offset_corr(shank1))
    PA_SERVO.angle(13, init_2h-90+ham2)
    PA_SERVO.angle(12, (init_2s+90)-mechan_offset_corr(shank2))
    PA_SERVO.angle(10, init_3h-90+ham3)
    PA_SERVO.angle(11, (init_3s+90)-mechan_offset_corr(shank3))
    PA_SERVO.angle(5, init_4h+90-ham4)
    PA_SERVO.angle(4, (init_4s-90)+mechan_offset_corr(shank4))
    # 记录最后输出的角度，供 smooth_transition 读取起始位
    last_servo_positions = [ham1, ham2, ham3, ham4, shank1, shank2, shank3, shank4]
  else:
    PA_SERVO.angle(2, init_1h); PA_SERVO.angle(3, init_1s)
    PA_SERVO.angle(13, init_2h); PA_SERVO.angle(12, init_2s)
    PA_SERVO.angle(10, init_3h); PA_SERVO.angle(11, init_3s)
    PA_SERVO.angle(5, init_4h); PA_SERVO.angle(4, init_4s)

def height(goal): global H_goal; H_goal=goal
def gesture(PIT,ROL,X): global PIT_goal,ROL_goal,X_goal; PIT_goal=PIT; ROL_goal=ROL; X_goal=X
def g(PIT): global PIT_goal; PIT_goal=PIT
def m(spd_,L_,R_): global spd,L,R; spd=spd_; L=L_; R=R_

def move(spd_,L_,R_):
    global spd_goal,L,R, current_pose
    if (current_pose == 'sit' or current_pose == 'lie') and spd_ != 0:
        stand_pose()
    spd_goal=spd_; L=L_; R=R_

def stable(key):
    global key_stab,speed
    key_stab=key
    if key==True: speed=speed_init+0.01
    else: speed=speed_init
  
def servo_init(key): global init_case; init_case=key
def gait(mode): global gait_mode; gait_mode=mode

#======V1.4 动作控制函数======

def s_curve(t):
    """公共 S 曲线插值函数，t 范围 [0, 1]，返回 [0, 1]"""
    if t <= 0: return 0.0
    if t >= 1: return 1.0
    return 6*t**5 - 15*t**4 + 10*t**3

def _relock():
    """重新锁定 mainloop（在动作内部调用 stand_pose 等姿态函数后使用）"""
    global stop_run_node
    stop_run_node = 1

def action_lock(func):
    """装饰器：自动管理 stop_run_node 锁定/解锁和异常处理"""
    def wrapper(*args, **kwargs):
        global stop_run_node
        stop_run_node = 1
        try:
            return func(*args, **kwargs)
        except Exception as e:
            print(func.__name__, "动作错误:", str(e))
        finally:
            stand_pose()
            stop_run_node = 0
    return wrapper

def smooth_transition(target_positions, steps=25, delay=0.04, velocity_profile='s-curve'):
    """平滑过渡函数：从当前姿态(last_servo_positions)平滑过渡到目标姿态

    参数:
        target_positions: 目标位置列表(8个舵机角度)
        steps: 过渡步数(默认25步)
        delay: 每步延迟时间(默认0.04秒)
        velocity_profile: 速度曲线类型('linear', 's-curve', 'trapezoidal')
    """
    try:
        # 使用 last_servo_positions 获取起始位置（由 servo_output 自动记录）
        if last_servo_positions is not None:
            start_positions = list(last_servo_positions)
        else:
            # 首次调用，计算默认站立位
            P_G = PA_ATTITUDE.cal_ges(0, 0, l, b, w, X_S, 110)
            start_positions = PA_IK.ik(ma_case, l1, l2,
                                       P_G[0], P_G[1], P_G[2], P_G[3],
                                       P_G[4], P_G[5], P_G[6], P_G[7])

        # 选择插值方法（统一使用模块级 s_curve）
        if velocity_profile == 's-curve':
            interp = s_curve
        elif velocity_profile == 'trapezoidal':
            def interp(x):
                if x < 0.2: return 2.5 * x * x
                elif x < 0.8: return 0.1 + (x - 0.2) * (4.0/3.0)
                else: return 1.0 - 2.5 * (1.0 - x) * (1.0 - x)
        else:
            def interp(x): return x

        # 执行平滑过渡
        for step in range(steps + 1):
            ratio = interp(step / steps)

            # 计算并应用中间位置
            pos = [start_positions[i] + (target_positions[i] - start_positions[i]) * ratio for i in range(8)]
            servo_output(ma_case, 0, pos[0], pos[1], pos[2], pos[3], pos[4], pos[5], pos[6], pos[7])
            time.sleep(delay)

    except Exception as e:
        print("平滑过渡错误:", str(e))
        servo_output(ma_case, 0,
                    target_positions[0], target_positions[1],
                    target_positions[2], target_positions[3],
                    target_positions[4], target_positions[5],
                    target_positions[6], target_positions[7])


def stand_pose():
    """站立姿态：所有腿高度110mm，并使用当前的重心偏移"""
    global H_goal, PIT_goal, ROL_goal, X_goal, current_pose, stop_run_node
    was_locked = (stop_run_node == 1)  # 检查是否已被 @action_lock 锁定
    stop_run_node = 1 # 锁定mainloop
    try:

        original_pose = current_pose

        H_goal = 110
        PIT_goal = PIT_S * 0.3  
        ROL_goal = ROL_S * 0.3
        X_goal = X_S

        P_G = PA_ATTITUDE.cal_ges(PIT_goal, ROL_goal, l, b, w, X_goal, H_goal)

        stand_positions = PA_IK.ik(ma_case, l1, l2,
                                   P_G[0], P_G[1], P_G[2], P_G[3],
                                   P_G[4], P_G[5], P_G[6], P_G[7])

        smooth_transition(stand_positions, steps=20, delay=0.035, velocity_profile='s-curve')

        current_pose = 'stand'
        PIT_goal = 0
        ROL_goal = 0

        time.sleep(0.2)
    except Exception as e:
        print("站立姿态逆解算错误:", str(e))
    finally:
        if not was_locked:
            stop_run_node = 0 # 仅在非嵌套调用时解锁


def sit_pose():
    """坐下姿态：前腿110mm，后腿90mm"""
    global current_pose, stop_run_node, H_goal, PIT_goal, ROL_goal, X_goal
    was_locked = (stop_run_node == 1)
    stop_run_node = 1
    try:
        original_pose = current_pose

        H_goal = 110
        PIT_goal = PIT_S * 0.2
        ROL_goal = ROL_S * 0.2
        X_goal = X_S

        P_G_sit = PA_ATTITUDE.cal_ges(PIT_goal, ROL_goal, l, b, w, X_goal, H_goal)

        sit_positions = PA_IK.ik(ma_case, l1, l2,
                               P_G_sit[0], P_G_sit[1], P_G_sit[2], P_G_sit[3],
                               -110, -110, -90, -90)

        smooth_transition(sit_positions, steps=18, delay=0.03, velocity_profile='s-curve')

        current_pose = 'sit'
        H_goal = 90
        PIT_goal = 0
        ROL_goal = 0

        time.sleep(0.15)
    except Exception as e:
        print("坐下姿态逆解算错误:", str(e))
    finally:
        if not was_locked:
            stop_run_node = 0


def lie_pose():
    """趴下姿态：所有腿高度90mm"""
    global current_pose, stop_run_node, H_goal, PIT_goal, ROL_goal, X_goal
    was_locked = (stop_run_node == 1)
    stop_run_node = 1
    try:
        original_pose = current_pose

        H_goal = 90   
        PIT_goal = PIT_S * 0.1  
        ROL_goal = ROL_S * 0.1
        X_goal = X_S  

        P_G_lie = PA_ATTITUDE.cal_ges(PIT_goal, ROL_goal, l, b, w, X_goal, H_goal)

        lie_positions = PA_IK.ik(ma_case, l1, l2,
                               P_G_lie[0], P_G_lie[1], P_G_lie[2], P_G_lie[3],
                               -90, -90, -90, -90)

        smooth_transition(lie_positions, steps=15, delay=0.025, velocity_profile='s-curve')

        current_pose = 'lie'
        PIT_goal = 0
        ROL_goal = 0

        time.sleep(0.1)
    except Exception as e:
        print("趴下姿态逆解算错误:", str(e))
    finally:
        if not was_locked:
            stop_run_node = 0

@action_lock
def greet_action():
    """(优化版)打招呼动作：平滑过渡，并带有身体协同"""
    # --- 动作参数 ---
    transition_steps = 20  # 姿态过渡的平滑步数
    wave_cycles = 3        # 挥手次数
    wave_speed_delay = 0.02 # 挥手时每一步的延迟，越小越快
    body_roll_angle = 5    # 挥手时身体侧倾的最大角度
    greet_leg_forward = 50 # 打招呼腿向前伸出的距离

    stand_pos_y = [-110, -110, -110, -110]
    greet_pos_y = [-110, -50, -90, -90]

    # 1. 停止当前运动
    m(0, 0, 0)
    time.sleep(0.2)
    
    stand_pose()
    time.sleep(0.1)

    # 2. 从"站立"平滑过渡到"准备打招呼"，使用S曲线速度控制
    print("过渡到打招呼姿态...")

    for i in range(1, transition_steps + 1):
        ratio = s_curve(i / transition_steps)  # 使用公共 s_curve

        # 计算当前步骤的中间Y坐标
        current_y = [
            stand_pos_y[j] + (greet_pos_y[j] - stand_pos_y[j]) * ratio for j in range(4)
        ]
        # 身体稍微后仰，为前腿抬起做准备
        current_pitch = -3 * ratio
        
        # 计算X坐标：支撑腿保持在X_S，招呼腿向前伸出
        leg_x = [X_S, X_S + greet_leg_forward * ratio, X_S, X_S]
        
        P_G = PA_ATTITUDE.cal_ges(current_pitch, 0, l, b, w, X_S, R_H)

        positions = PA_IK.ik(ma_case, l1, l2, 
                           leg_x[0] + P_G[0], leg_x[1] + P_G[1], 
                           leg_x[2] + P_G[2], leg_x[3] + P_G[3],
                           current_y[0] + (P_G[4] + 110),
                           current_y[1] + (P_G[5] + 110),
                           current_y[2] + (P_G[6] + 110),
                           current_y[3] + (P_G[7] + 110))
                           
        servo_output(ma_case, 0,
                    positions[0], positions[1], positions[2], positions[3],
                    positions[4], positions[5], positions[6], positions[7])
        time.sleep(0.03)

    # 3. 挥手并伴随身体侧倾
    print("开始挥手...")
    for _ in range(wave_cycles):
        for i in range(60):
            angle = i * (pi / 30)
            wave_x = greet_leg_forward + 20 * sin(angle)
            body_roll = body_roll_angle * sin(angle)
            leg_x = [X_S, X_S + wave_x, X_S, X_S]
            
            P_G = PA_ATTITUDE.cal_ges(current_pitch, body_roll, l, b, w, X_S, R_H)

            positions = PA_IK.ik(ma_case, l1, l2, 
                               leg_x[0] + P_G[0], leg_x[1] + P_G[1], 
                               leg_x[2] + P_G[2], leg_x[3] + P_G[3],
                               greet_pos_y[0] + (P_G[4] + 110),
                               greet_pos_y[1] + (P_G[5] + 110),
                               greet_pos_y[2] + (P_G[6] + 110),
                               greet_pos_y[3] + (P_G[7] + 110))
                               
            servo_output(ma_case, 0,
                    positions[0], positions[1], positions[2], positions[3],
                    positions[4], positions[5], positions[6], positions[7])
            time.sleep(wave_speed_delay)

    print("恢复站立姿态...")

def forward_3s_action():
    """前进3秒：以速度3前进3秒后自动停止"""
    global timed_action_end_time, timed_action_running
    try:
        # 确保站立姿态开始
        if current_pose != 'stand':
            stand_pose()
            time.sleep(0.3)  # 减少等待时间，因为站立动作现在有平滑过渡

        # 设置前进速度3
        move(3, 1, 1)
        timed_action_end_time = time.ticks_add(time.ticks_ms(), 3000)
        timed_action_running = True
    except Exception as e:
        print("前进3秒动作错误:", str(e))

def backward_3s_action():
    """后退3秒：以速度-3后退3秒后自动停止"""
    global timed_action_end_time, timed_action_running
    try:
        # 确保站立姿态开始
        if current_pose != 'stand':
            stand_pose()
            time.sleep(0.3)  # 减少等待时间

        # 设置后退速度-3，使用正确的参数
        move(-3, 1, 1)  # 修正：L和R应为1,1，速度为负表示后退
        timed_action_end_time = time.ticks_add(time.ticks_ms(), 3000)
        timed_action_running = True
    except Exception as e:
        print("后退3秒动作错误:", str(e))

def left_3s_action():
    """左转3秒：以速度2.5左转3秒后自动停止"""
    global timed_action_end_time, timed_action_running
    try:
        # 确保站立姿态开始
        if current_pose != 'stand':
            stand_pose()
            time.sleep(0.3)  # 减少等待时间

        # 设置左转
        move(2.5, -1, 1)
        timed_action_end_time = time.ticks_add(time.ticks_ms(), 3000)
        timed_action_running = True
    except Exception as e:
        print("左转3秒动作错误:", str(e))

def right_3s_action():
    """右转3秒：以速度2.5右转3秒后自动停止"""
    global timed_action_end_time, timed_action_running
    try:
        # 确保站立姿态开始
        if current_pose != 'stand':
            stand_pose()
            time.sleep(0.3)  # 减少等待时间

        # 设置右转
        move(2.5, 1, -1)
        timed_action_end_time = time.ticks_add(time.ticks_ms(), 3000)
        timed_action_running = True
    except Exception as e:
        print("右转3秒动作错误:", str(e))

#======V1.4 新增及优化的动作函数======

@action_lock
def dance():
    """(优化版) 波浪舞蹈：带渐入渐出缓冲，四条腿依次起伏形成平滑波浪效果"""
    # 停止当前运动
    m(0, 0, 0)

    # --- 波浪舞蹈参数 ---
    wave_height = 12       # 波浪高度 (mm)
    wave_cycles = 4        # 完整波浪循环次数
    steps_per_cycle = 100  # 一个周期(2*pi)的步数
    delay_per_step = 0.02  # 每步延迟
    fade_steps = 30        # 渐入/渐出步数
    # ---

    # 初始站立姿态（stand_pose 现在不会解锁 stop_run_node，因为 @action_lock 已锁定）
    stand_pose()
    time.sleep(0.1)
    
    # 精确过渡到舞蹈起始帧（pitch=0, roll=0, height=110）
    P_G_start = PA_ATTITUDE.cal_ges(0, 0, l, b, w, X_S, 110)
    start_dance_positions = PA_IK.ik(ma_case, l1, l2,
                                     P_G_start[0], P_G_start[1], P_G_start[2], P_G_start[3],
                                     P_G_start[4], P_G_start[5], P_G_start[6], P_G_start[7])
    smooth_transition(start_dance_positions, steps=10, delay=0.02)

    # 内部函数：计算单帧舞蹈姿态并输出
    def _dance_frame(phase, cycle_idx, wh):
        leg1_h = -110 + wh * sin(phase)
        leg2_h = -110 + wh * sin(phase + pi/2)
        leg3_h = -110 + wh * sin(phase + pi)
        leg4_h = -110 + wh * sin(phase + 3*pi/2)
        # 身体姿态协同，幅度随 wh 缩放
        scale = wh / wave_height if wave_height > 0 else 0
        pitch_angle = 5 * sin(cycle_idx * 2 * pi / wave_cycles + phase) * (1 - abs(cos(phase))) * scale
        roll_angle = 3 * cos(phase) * scale
        P_G = PA_ATTITUDE.cal_ges(pitch_angle, roll_angle, l, b, w, X_S, 110)
        positions = PA_IK.ik(ma_case, l1, l2,
                           P_G[0], P_G[1], P_G[2], P_G[3],
                           P_G[4] + (leg1_h + 110),
                           P_G[5] + (leg2_h + 110),
                           P_G[6] + (leg3_h + 110),
                           P_G[7] + (leg4_h + 110))
        servo_output(ma_case, 0,
                    positions[0], positions[1], positions[2], positions[3],
                    positions[4], positions[5], positions[6], positions[7])

    print("开始舞蹈...")

    total_fade  = fade_steps                        # 渐入帧数
    total_main  = wave_cycles * steps_per_cycle     # 主循环帧数
    total_fade2 = fade_steps                        # 渐出帧数
    total_steps = total_fade + total_main + total_fade2

    for global_step in range(total_steps):
        # phase 全程单调递增，保证 sin 连续
        phase = global_step * 2 * pi / steps_per_cycle

        if global_step < total_fade:
            # 渐入：幅度从 0 → wave_height
            wh = wave_height * s_curve(global_step / total_fade)
            cycle_idx = 0
        elif global_step < total_fade + total_main:
            # 正常舞蹈
            wh = wave_height
            cycle_idx = (global_step - total_fade) // steps_per_cycle
        else:
            # 渐出：幅度从 wave_height → 0
            fade_pos = global_step - total_fade - total_main
            wh = wave_height * s_curve(1.0 - fade_pos / total_fade2)
            cycle_idx = wave_cycles - 1

        _dance_frame(phase, cycle_idx, wh)
        time.sleep(delay_per_step)

    print("舞蹈结束，恢复站立。")

@action_lock
def stretch():
    """伸展 (伸懒腰) 动作"""
    print("执行动作: 伸展 (缓慢)")
    # 1. 确保在站立姿态
    if current_pose != 'stand':
        stand_pose()
        time.sleep(0.3)
    
    # 2. 计算目标姿态：前腿前伸并降低，后腿抬高
    stretch_x = [X_S + 50, X_S + 50, X_S, X_S]
    stretch_y = [-95, -95, -115, -115]
    
    # 3. 计算伸展时的舵机角度
    stretch_positions = PA_IK.ik(ma_case, l1, l2,
                               stretch_x[0], stretch_x[1], stretch_x[2], stretch_x[3],
                               stretch_y[0], stretch_y[1], stretch_y[2], stretch_y[3])
    
    # 4. 去程：S曲线过渡到伸展姿态
    smooth_transition(stretch_positions, steps=40, delay=0.03, velocity_profile='s-curve')
    
    # 5. 保持伸展
    time.sleep(2.0)
    
    # 6. 回程：smooth_transition 现在能自动从 last_servo_positions 过渡回站立位
    print("伸展结束，缓慢恢复...")
    P_G_stand = PA_ATTITUDE.cal_ges(0, 0, l, b, w, X_S, 110)
    stand_positions = PA_IK.ik(ma_case, l1, l2,
                               P_G_stand[0], P_G_stand[1], P_G_stand[2], P_G_stand[3],
                               P_G_stand[4], P_G_stand[5], P_G_stand[6], P_G_stand[7])
    smooth_transition(stand_positions, steps=40, delay=0.03, velocity_profile='s-curve')
        
        
@action_lock
def push_up():
    """俯卧撑动作：S曲线过渡"""
    print("执行动作: 俯卧撑")
    push_up_cycles = 3
    
    # 1. S曲线过渡到趴下姿态
    lie_pose()
    time.sleep(0.3)
    
    # 2. 定义低位和高位
    P_G_low = PA_ATTITUDE.cal_ges(0, 0, l, b, w, X_S, 90)
    low_positions = PA_IK.ik(ma_case, l1, l2,
                             P_G_low[0], P_G_low[1], P_G_low[2], P_G_low[3],
                             P_G_low[4], P_G_low[5], P_G_low[6], P_G_low[7])
    
    P_G_high = PA_ATTITUDE.cal_ges(0, 0, l, b, w, X_S, 110)
    high_positions = PA_IK.ik(ma_case, l1, l2,
                              P_G_high[0], P_G_high[1], P_G_high[2], P_G_high[3],
                              P_G_high[4], P_G_high[5], P_G_high[6], P_G_high[7])
                              
    # 3. 循环执行俯卧撑
    for _ in range(push_up_cycles):
        smooth_transition(high_positions, steps=15, delay=0.025, velocity_profile='s-curve')
        time.sleep(0.3)
        smooth_transition(low_positions, steps=15, delay=0.025, velocity_profile='s-curve')
        time.sleep(0.3)
    
    print("俯卧撑结束，恢复站立。")

@action_lock
def shake_body():
    """摇晃身体 (优化版)：适配高摩擦力和高重心"""
    print("执行动作: 摇晃身体 (防侧翻)")
    # 1. 先站稳
    stand_pose()
    time.sleep(0.3)
    
    # --- 参数调整 ---
    shake_cycles = 4
    steps_per_cycle = 60
    delay = 0.025
    max_roll = 8
    # ----------------
    
    for _ in range(shake_cycles):
        for step in range(steps_per_cycle):
            phase = step * 2 * pi / steps_per_cycle
            current_roll = max_roll * sin(phase)
            P_G = PA_ATTITUDE.cal_ges(0, current_roll, l, b, w, X_S, R_H)
            positions = PA_IK.ik(ma_case, l1, l2,
                               P_G[0], P_G[1], P_G[2], P_G[3],
                               P_G[4], P_G[5], P_G[6], P_G[7])
            servo_output(ma_case, 0, 
                       positions[0], positions[1], positions[2], positions[3],
                       positions[4], positions[5], positions[6], positions[7])
            time.sleep(delay)
    
    print("摇晃结束，恢复站立。")

#======V1.4 动作函数结束======

def swing_curve_generate(t,Tf,xt,zh,x0,z0,xv0):
  #输入参数：当前时间；支撑相占空比；x方向目标位置，z方向抬腿高度，摆动相抬腿前腿速度
  # X Generator
  if t>=0 and t<Tf/4:
    xf=(-4*xv0/Tf)*t*t+xv0*t+x0
    
  if t>=Tf/4 and t<(3*Tf)/4:
    xf=((-4*Tf*xv0-16*xt+16*x0)*t*t*t)/(Tf*Tf*Tf)+((7*Tf*xv0+24*xt-24*x0)*t*t)/(Tf*Tf)+((-15*Tf*xv0-36*xt+36*x0)*t)/(4*Tf)+(9*Tf*xv0+16*xt)/(16)
    
  if t>(3*Tf)/4:
    xf=xt

  # Z Generator
  if t>=0 and t<Tf/2:
    zf=(16*z0-16*zh)*t*t*t/(Tf*Tf*Tf)+(12*zh-12*z0)*t*t/(Tf*Tf)+z0
  
  if t>=Tf/2:
    zf=(4*z0-4*zh)*t*t/(Tf*Tf)-(4*z0-4*zh)*t/(Tf)+z0
      
  #Record touch down position
  x_past=xf
  t_past=t
  
  # # Avoid zf to go zero
  if zf<=0:
    zf=0
  #x,z position,x_axis stop point,t_stop point;depend on when the leg stop
  
  return xf,zf,x_past,t_past


def support_curve_generate(t,Tf,x_past,t_past,zf):
  # 当前时间；支撑相占空比；摆动相 x 最终位;t最终时间，支撑相 zf
  # Only X Generator
  average=x_past/(1-Tf)
  xf=x_past-average*(t-t_past)
  return xf,zf
  
def alarm_and_servo_control():
  global normal_node,error_node,empty_power_count, has_printed_initial_voltage
  judge_num_node=0
  
  # 读取电压并计算
  val = adc.read()
  voltage = read_voltage(val)
  
  # 仅在上电时打印一次电压信息，防止刷屏
  if not has_printed_initial_voltage:
      print(f"系统启动 - 当前电池电压: {voltage:.2f}V (Raw: {val})")
      has_printed_initial_voltage = True

  # 低压检测
  if voltage < 5.5 and voltage > 0.8:  
    empty_power_count+=1
  else:
    empty_power_count=0
    
  if empty_power_count>=200:
    judge_num_node+=1; empty_power_count=200; alarm(200,1000,0)
  
  # USB 保护 (如果开启调试模式，则不报警)
  if voltage < 0.8: 
      if not DEBUG_USB_MODE:
          judge_num_node+=1
      else:
          # USB调试模式下，不增加 judge_num_node，只打印警告一次
          pass

  if IK_ERROR==1: judge_num_node+=1
  
  if IK_ERROR==1 and error_node==0:
    alarm(300,100,0); error_node=1; normal_node=0
    print("逆解算错误")
    
  if IK_ERROR==0 and normal_node==0:
    alarm(0,0,0); error_node=0; normal_node=1
    
  return judge_num_node
      
def mainloop():
  global t, R_H, PIT_S, ROL_S, X_S, act_tran_mov_kp, PIT_goal, ROL_goal, spd
  global ges_x_1, ges_x_2, ges_x_3, ges_x_4, ges_y_1, ges_y_2, ges_y_3, ges_y_4, IK_ERROR
  global timed_action_running, timed_action_end_time, current_pose

  if stop_run_node==1: return 0
  
  # 静态姿态检查
  if current_pose == 'sit' or current_pose == 'lie':
    if alarm_and_servo_control() != 0: 
        PA_SERVO.release(); pin_servo_vol.value(0)
    else: 
        pin_servo_vol.value(1)
    return

  # 定时动作结束检查
  if timed_action_running and time.ticks_diff(time.ticks_ms(), timed_action_end_time) > 0:
    move(0, 0, 0); timed_action_running = False
  
  # 舵机电源控制
  if alarm_and_servo_control()==0: pin_servo_vol.value(1)
  else: PA_SERVO.release(); pin_servo_vol.value(0)
  
  # === 恢复完整的步态解算逻辑 ===
  if gait_mode==0:
    act_tran_mov_kp=tran_mov_kp
    if t>=1: t=0
    elif L==0 and R==0: t=0
    else: t=t+speed
    P_=PA_GAIT.trot(t,spd*10,h,L,L,R,R)
  elif gait_mode==1:
    act_tran_mov_kp=tran_mov_kp/2
    P_=PA_GAIT.walk(t,spd*10,h,L,L,R,R)
    if t>=2.5: t=0
    elif L==0 and R==0: t=0
    else: pass

  # PID 调节
  if spd>spd_goal: spd=spd-abs(spd-spd_goal)*Kp_V
  elif spd<spd_goal: spd=spd+abs(spd-spd_goal)*Kp_V
  
  if R_H>H_goal: R_H=R_H-abs(R_H-H_goal)*Kp_H
  elif R_H<H_goal: R_H=R_H+abs(R_H-H_goal)*Kp_H
  
  if key_stab!=True:
    if PIT_S>PIT_goal: PIT_S=PIT_S-abs(PIT_S-PIT_goal)*pit_Kp_G
    elif PIT_S<PIT_goal: PIT_S=PIT_S+abs(PIT_S-PIT_goal)*pit_Kp_G
    if ROL_S>ROL_goal: ROL_S=ROL_S-abs(ROL_S-ROL_goal)*rol_Kp_G
    elif ROL_S<ROL_goal: ROL_S=ROL_S+abs(ROL_S-ROL_goal)*rol_Kp_G
  else:
    PIT_S=PIT_goal; ROL_S=ROL_goal

  if X_S>X_goal: X_S=X_S-abs(X_S-X_goal)*act_tran_mov_kp
  elif X_S<X_goal: X_S=X_S+abs(X_S-X_goal)*act_tran_mov_kp

  if PIT_S>=pit_max_ang:PIT_S=pit_max_ang
  if PIT_S<=-pit_max_ang:PIT_S=-pit_max_ang
  if ROL_S>=rol_max_ang:ROL_S=rol_max_ang
  if ROL_S<=-rol_max_ang:ROL_S=-rol_max_ang

  if gait_mode==0:
    if spd>=0 and (L+R)!=0: P_G=PA_ATTITUDE.cal_ges(PIT_S,ROL_S,l,b,w,X_S+abs(spd)*trot_cg_f,R_H)
    elif spd<0 and (L+R)!=0: P_G=PA_ATTITUDE.cal_ges(PIT_S,ROL_S,l,b,w,X_S-abs(spd)*trot_cg_b,R_H)
    elif (L+R)==0: P_G=PA_ATTITUDE.cal_ges(PIT_S,ROL_S,l,b,w,X_S+abs(spd)*trot_cg_t,R_H)
  else:
    P_G=PA_ATTITUDE.cal_ges(PIT_S,ROL_S,l,b,w,X_S,R_H)
    
  ges_x_1=P_G[0];ges_x_2=P_G[1]; ges_x_3=P_G[2]; ges_x_4=P_G[3]
  ges_y_1=P_G[4];ges_y_2=P_G[5]; ges_y_3=P_G[6]; ges_y_4=P_G[7]

  if key_stab==True:
    if abs(0-spd)<=0.1 and L==0 and R==0: PA_STABLIZE.stab()
    else: PIT_goal=0; ROL_goal=0

  try:
    # 真实的 IK 解算
    A_=PA_IK.ik(ma_case,l1,l2,P_[0]+ges_x_1,P_[1]+ges_x_2,P_[2]+ges_x_3,P_[3]+ges_x_4,P_[4]+ges_y_1,P_[5]+ges_y_2,P_[6]+ges_y_3,P_[7]+ges_y_4)
    servo_output(ma_case,init_case,A_[0],A_[1],A_[2],A_[3],A_[4],A_[5],A_[6],A_[7])
    IK_ERROR=0
  except:
    IK_ERROR=1
    print("Mainloop IK Error")
print("伴伴智宠 - Ban-Ban Dog")
print("版本：V2.0")