# 文件名: main.py
from machine import Timer, freq, Pin
import time
from PA_SERVO import release
import padog
import _thread

release()
freq(240000000)

# ================= 启动模式选择 =================
boot_btn = Pin(0, Pin.IN, Pin.PULL_UP)
#led = Pin(22, Pin.OUT) # 板载 LED 是 GPIO 22

print("等待启动模式选择 (3秒)...")
mode_debug = False

# 闪烁 LED 提示用户可以按键
for i in range(6): 
    #led.value(not led.value())
    if boot_btn.value() == 0: # 按下为低电平
        mode_debug = True
        print("检测到按键 -> 进入 WiFi 调试模式")
        padog.alarm(100, 800, 800)
        time.sleep(0.2)
        padog.alarm(0,0,0)
        break
    time.sleep(0.3)
#led.value(1) # 常亮表示启动完成

# ================= 主循环逻辑 =================
t = Timer(1)

def loop(t):
    try:
        padog.mainloop()     # 核心运动解算
        padog.alarm_run()    # 报警检测
        
        # 无论哪种模式，都可以保留串口检测 (作为备用或语音控制)
        padog.UART_Run() 
        
        # 原有的高度和重心维持逻辑
        if padog.CC_M != 0: # 如果不是纯网页模式
             padog.height(110)
             padog.X_goal = padog.in_y

        # 循环变速逻辑
        if padog.loop_speed_mode_sc==1:
            if padog.loop_speed_mode==0:
                t.deinit()
                t.init(period=5,mode=Timer.PERIODIC,callback=loop)
                padog.loop_speed_mode_sc=0
            elif padog.loop_speed_mode==1:
                t.deinit()
                t.init(period=5,mode=Timer.PERIODIC,callback=loop)
                padog.loop_speed_mode_sc=0
    except Exception as e:
        print("Fatal Error:", e)
        t.deinit()

# ================= 根据模式启动服务 =================

def start_web_server():
    import web_c
    web_c.run_web_server()

def start_ble_server():
    import ble_server
    # 初始化蓝牙，传入实例名称
    ble = ble_server.BLEServer("BanBan-Dog")

# 初始化定时器
t.init(period=5, mode=Timer.PERIODIC, callback=loop)
padog.loop_speed_mode_sc = 0
padog.start_ring() # 开机音效

if mode_debug:
    # --- 调试模式 (WiFi + Web) ---
    print("模式: WEB 调试 & 标定")
    padog.CC_M = 0 # 标记为网页控制模式
    padog.do_connect_AP() # 阻塞连接热点
    _thread.stack_size(1024*8)
    _thread.start_new_thread(start_web_server, ())
    
else:
    # --- 用户模式 (Bluetooth + UART Voice) ---
    print("模式: 蓝牙 APP & 语音串口")
    padog.CC_M = 2 # 标记为指令模式
    # 启动蓝牙 (非阻塞，基于中断)
    start_ble_server()
    padog.alarm(200, 400, 600) # 提示音
    time.sleep(0.5) 
    padog.alarm(0, 0, 0)
