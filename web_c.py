import padog
import socket
from math import floor
import time
#-----------------------HTTP Server-----------------------#
user_leg_num="1"
url_cal="cal.html"
url_c="control.html"
url_v= "voice_mode.html"  # 语音模式网页路径
thr=0;turn=0;L=0;R=0;Pitch=0;Roll=0;Yst=padog.in_y;Hgt=110 #中间变量定义
color_leg1='#7DFF7D';color_leg2='#FF9E9E';color_leg3='#FF9E9E';color_leg4='#FF9E9E';
step_state=0;test_add=0
url_n = url_v

def run_web_server():
    """优化的Web服务器函数，减少内存使用"""
    global user_leg_num, url_cal, url_c, url_v, thr, turn, L, R, Pitch, Roll, Yst, Hgt
    global color_leg1, color_leg2, color_leg3, color_leg4, step_state, test_add, url_n
    
    # 初始化变量
    user_leg_num = "1"
    url_cal = "cal.html"
    url_c = "control.html"
    url_v = "voice_mode.html"
    thr = 0
    turn = 0
    L = 0
    R = 0
    Pitch = 0
    Roll = 0
    Yst = padog.in_y
    Hgt = 110
    color_leg1 = '#7DFF7D'
    color_leg2 = '#FF9E9E'
    color_leg3 = '#FF9E9E'
    color_leg4 = '#FF9E9E'
    step_state = 0
    test_add = 0
    url_n = url_v
    
    addr = (str(padog.selfadd),80) #定义socket绑定的地址，ip地址为本地，端口为80
    s = socket.socket()     #创建一个socket对象
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)  # 允许地址重用
    s.bind(addr)            #绑定地址
    s.listen(1)             #设置允许连接的客户端数量
    print('控制网页地址:', addr)
    padog.gesture(Pitch,Roll,Yst)

    while True:
        try:
            cl, addr = s.accept() #接受客户端的连接请求
            req=str(cl.recv(1024))
            
            # 简化响应头，减少内存使用
            cl.sendall(b'HTTP/1.1 200 OK\nConnection: close\nServer: FireBeetle\nContent-Type: text/html\n\n')
            
            req=req.split('\\r\\n')
            #http header 解析
            req_data=req[0].lstrip().rstrip().replace(' ','').lower()
            if req_data.find('favicon.ico')>-1:
                cl.close()
                continue
            
            req_data=req_data.replace('get/?','').replace('http/1.1','').replace("b'","")
            print('req_data',req_data)
            
            # 处理参数
            if req_data.find('speed')>-1:
                try:
                    # 解析参数
                    params = {}
                    pairs = req_data.split('&')
                    for pair in pairs:
                        if '=' in pair:
                            key, value = pair.split('=', 1)
                            try:
                                params[key] = float(value)
                            except:
                                params[key] = value
                    
                    # 更新padog参数
                    if 'l1' in params: padog.l1 = float(params['l1'])
                    if 'l2' in params: padog.l2 = float(params['l2'])
                    if 'l' in params: padog.l = float(params['l'])
                    if 'b' in params: padog.b = float(params['b'])
                    if 'w' in params: padog.w = float(params['w'])
                    if 'speed' in params: 
                        padog.speed = float(params['speed'])
                        padog.speed_init = float(params['speed'])
                    if 'h' in params: padog.h = float(params['h'])
                    if 'kp_h' in params: padog.Kp_H = float(params['kp_h'])
                    if 'pit_kp_g' in params: padog.pit_Kp_G = float(params['pit_kp_g'])
                    if 'pit_kd_g' in params: padog.pit_Kd_G = float(params['pit_kd_g'])
                    if 'rol_kp_g' in params: padog.rol_Kp_G = float(params['rol_kp_g'])
                    if 'rol_kd_g' in params: padog.rol_Kd_G = float(params['rol_kd_g'])
                    if 'tran_mov_kp' in params: padog.tran_mov_kp = float(params['tran_mov_kp'])
                    if 'cc_m' in params: padog.CC_M = int(float(params['cc_m']))
                    if 'trot_cg_f' in params: padog.trot_cg_f = float(params['trot_cg_f'])
                    if 'trot_cg_b' in params: padog.trot_cg_b = float(params['trot_cg_b'])
                    if 'trot_cg_t' in params: padog.trot_cg_t = float(params['trot_cg_t'])
                except Exception as e:
                    print("参数处理错误:", str(e))
                    pass
            
            # 处理摇杆和按钮
            try:
                # 判断摇杆
                index_f = req_data.find('f=')
                index_find_t = req_data.find('t=')
                if index_f != -1 and index_find_t != -1:
                    value_f = req_data[index_f+2:index_find_t].lstrip().rstrip()
                    index_t = req_data.find('t=')
                    value_t = req_data[index_t+2:index_t+6].lstrip().rstrip()
                    thr=int(value_f)*6/100
                    turn=int(value_t)
                
                # 判断按钮
                index = req_data.find('key=')
                if index != -1:
                    value = req_data[index+4:index+6].lstrip().rstrip()
                    
                    # 运动控制用
                    if value == 'ss':
                        Pitch=0;Roll=0
                        padog.gait(0)
                        url_n=url_cal
                        padog.loop_speed_mode=1
                        padog.loop_speed_mode_sc=1
                        step_state=0
                    elif value == 'go':
                        padog.stable(True)
                    elif value == 'gc':
                        padog.stable(False)
                    elif value == 'sn':
                        step_state=1
                    elif value == 'sf':
                        step_state=0
                    elif value == 'g0':
                        padog.alarm(50,500,600)
                        time.sleep(1)
                        padog.stable(False)
                        padog.gait(0)
                        padog.alarm(0,0,0)
                    elif value == 'g1':
                        padog.alarm(50,600,700)
                        time.sleep(1)
                        padog.stable(False)
                        padog.gait(1)
                        padog.alarm(0,0,0)
                    elif value == 'c2':
                        url_n = url_v
                        padog.CC_M = 2
                        print('切换到语音控制模式')
                    elif value == 'c0':
                        url_n = url_c
                        padog.CC_M = 0
                        print('切换回网页控制模式')
                    
                    # 标定判断用
                    elif value == 'l2':
                        user_leg_num='2'
                        color_leg1='#FF9E9E';color_leg2='#7DFF7D';color_leg3='#FF9E9E';color_leg4='#FF9E9E'
                    elif value == 'l4':
                        user_leg_num='4'
                        color_leg1='#FF9E9E';color_leg2='#FF9E9E';color_leg3='#FF9E9E';color_leg4='#7DFF7D'
                    elif value == 'l1':
                        user_leg_num='1'
                        color_leg1='#7DFF7D';color_leg2='#FF9E9E';color_leg3='#FF9E9E';color_leg4='#FF9E9E'
                    elif value == 'l3':
                        user_leg_num='3'
                        color_leg1='#FF9E9E';color_leg2='#FF9E9E';color_leg3='#7DFF7D';color_leg4='#FF9E9E'
                    elif value == 'sc':
                        s_f = open("config_s.py", "w+")
                        s_f.write("init_1h="+str(padog.init_1h)+"\n")
                        s_f.write("init_1s="+str(padog.init_1s)+"\n")
                        s_f.write("init_2h="+str(padog.init_2h)+"\n")
                        s_f.write("init_2s="+str(padog.init_2s)+"\n")
                        s_f.write("init_3h="+str(padog.init_3h)+"\n")
                        s_f.write("init_3s="+str(padog.init_3s)+"\n")
                        s_f.write("init_4h="+str(padog.init_4h)+"\n")
                        s_f.write("init_4s="+str(padog.init_4s)+"\n")
                        s_f.write("l1="+str(padog.l1)+"\n")
                        s_f.write("l2="+str(padog.l2)+"\n")
                        s_f.write("l="+str(padog.l)+"\n")
                        s_f.write("b="+str(padog.b)+"\n")
                        s_f.write("w="+str(padog.w)+"\n")
                        s_f.write("speed="+str(padog.speed)+"\n")
                        s_f.write("h="+str(padog.h)+"\n")
                        s_f.write("Kp_H="+str(padog.Kp_H)+"\n")
                        s_f.write("pit_Kp_G="+str(padog.pit_Kp_G)+"\n")
                        s_f.write("pit_Kd_G="+str(padog.pit_Kd_G)+"\n")
                        s_f.write("rol_Kp_G="+str(padog.rol_Kp_G)+"\n")
                        s_f.write("rol_Kd_G="+str(padog.rol_Kd_G)+"\n")
                        s_f.write("tran_mov_kp="+str(padog.tran_mov_kp)+"\n")
                        s_f.write("CC_M="+str(padog.CC_M)+"\n")
                        s_f.write("ma_case="+str(padog.ma_case)+"\n")
                        s_f.write("trot_cg_f="+str(padog.trot_cg_f)+"\n")
                        s_f.write("trot_cg_b="+str(padog.trot_cg_b)+"\n")
                        s_f.write("trot_cg_t="+str(padog.trot_cg_t)+"\n")
                        s_f.write("in_y="+str(Yst)+"\n")
                        s_f.close()
                        url_n=url_c
                        padog.loop_speed_mode=0
                        padog.loop_speed_mode_sc=1
                        padog.servo_init(0)
                    
                    elif value == 'hi':
                        try:
                            leg_num = int(user_leg_num)
                            if leg_num == 1:
                                padog.init_1h += 1
                            elif leg_num == 2:
                                padog.init_2h += 1
                            elif leg_num == 3:
                                padog.init_3h += 1
                            elif leg_num == 4:
                                padog.init_4h += 1
                        except:
                            pass
                    elif value == 'hd':
                        try:
                            leg_num = int(user_leg_num)
                            if leg_num == 1:
                                padog.init_1h -= 1
                            elif leg_num == 2:
                                padog.init_2h -= 1
                            elif leg_num == 3:
                                padog.init_3h -= 1
                            elif leg_num == 4:
                                padog.init_4h -= 1
                        except:
                            pass
                    elif value == 'si':
                        try:
                            leg_num = int(user_leg_num)
                            if leg_num == 1:
                                padog.init_1s += 1
                            elif leg_num == 2:
                                padog.init_2s += 1
                            elif leg_num == 3:
                                padog.init_3s += 1
                            elif leg_num == 4:
                                padog.init_4s += 1
                        except:
                            pass
                    elif value == 'sd':
                        try:
                            leg_num = int(user_leg_num)
                            if leg_num == 1:
                                padog.init_1s -= 1
                            elif leg_num == 2:
                                padog.init_2s -= 1
                            elif leg_num == 3:
                                padog.init_3s -= 1
                            elif leg_num == 4:
                                padog.init_4s -= 1
                        except:
                            pass
                    elif value == 't9':
                        padog.servo_init(1)
                
                # 处理姿态参数
                if req_data.find('pit=')>-1:
                    index_p = req_data.find('pit=')
                    value_p = req_data[index_p+4:index_p+7].lstrip().rstrip()
                    if value_p!='/':
                        Pitch=int(value_p)
                
                if req_data.find('rol=')>-1:
                    index_r = req_data.find('rol=')
                    value_r = req_data[index_r+4:index_r+7].lstrip().rstrip()
                    if value_r!='/':
                        Roll=int(value_r)
                
                if req_data.find('hgt=')>-1:
                    index_h = req_data.find('hgt=')
                    value_h = req_data[index_h+4:index_h+7].lstrip().rstrip()
                    if value_h!='/':
                        Hgt=int(value_h)
                
                if req_data.find('yst=')>-1:
                    index_y = req_data.find('yst=')
                    value_y = req_data[index_y+4:index_y+7].lstrip().rstrip()
                    if value_y!='/':
                        Yst=int(value_y)
                        
            except Exception as e:
                print("Web处理错误:", str(e))
            
            # 命令处理
            if thr>=3:
                thr=3
            elif thr<=-3:
                thr=-3
            if turn>=80:
                L=1;R=-1;thr=2
            elif turn<=-80:
                L=-1;R=1;thr=2
            else:
                L=1;R=1
            
            # 检查当前模式
            if padog.CC_M==0:
                if thr==0:
                    padog.spd_goal=0
                    if step_state==1:
                        padog.move(0,1,1)
                    elif abs(0-padog.spd)<=1:
                        padog.move(0,0,0)
                else:
                    padog.move(thr,L,R)

                padog.height(Hgt)
                padog.X_goal=Yst
                if padog.key_stab!=True:
                    padog.PIT_goal=Pitch
                    padog.ROL_goal=Roll
            
            # 发送响应内容
            try:
                with open(url_n, 'r') as f:
                    if req_data.find('speed')>-1 or (req_data.find('f=')==-1 and req_data.find('g0')==-1 and req_data.find('g1')==-1 and req_data.find('sn')==-1 and req_data.find('sf')==-1 and req_data.find('go')==-1 and req_data.find('gc')==-1 and req_data.find('pit=')==-1 and req_data.find('rol=')==-1 and req_data.find('yst=')==-1 and req_data.find('hgt=')==-1):
                        while True:
                            out=f.read(256)  # 减少每次读取的字节数
                            if out:
                                try:
                                    cl.sendall(out)
                                except:
                                    break
                            else:
                                break
                    
                    if url_n=="cal.html":
                        cl.sendall("""
                        <center><table border="3">
                        <tr>
                        <th bgcolor='"""+color_leg1+"""'>1号大腿："""+str(padog.init_1h)+"""<Br/>1号小腿："""+str(padog.init_1s)+"""</th>
                        <th bgcolor='"""+color_leg2+"""'>2号大腿："""+str(padog.init_2h)+"""<Br/>2号小腿："""+str(padog.init_2s)+"""</th>
                        </tr>
                        <tr>
                        <th bgcolor='"""+color_leg4+"""'>4号大腿："""+str(padog.init_4h)+"""<Br/>4号小腿："""+str(padog.init_4s)+"""</th>
                        <th bgcolor='"""+color_leg3+"""'>3号大腿："""+str(padog.init_3h)+"""<Br/>3号小腿："""+str(padog.init_3s)+"""</th>
                        </tr>
                        </table></center><Br/><Br/>
                        """)
                        
                        cl.sendall("""
                        <center>
                        <h1>控制器参数设定</h1>
                        <br />
                        <br />
                        <form action="/" method="get" accept-charset="utf-8">
                        <br /><p>大腿(杆1)长　　　　   :&nbsp;<input type="text"  style="width:100px; height:30px;" name="l1" """+"""value='"""+str(padog.l1)+"""'/></p>"""+
                        """<br /><p>小腿(杆2)长　　　　   :&nbsp;<input type="text"  style="width:100px; height:30px;" name="l2" """+"""value='"""+str(padog.l2)+"""'/></p>"""+
                        """<br /><p>机器人长度　　 　　   :&nbsp;<input type="text"  style="width:100px; height:30px;" name="l" """+"""value='"""+str(padog.l)+"""'/></p>"""+
                        """<br /><p>机器人宽度　　 　　   :&nbsp;<input type="text"  style="width:100px; height:30px;" name="b" """+"""value='"""+str(padog.b)+"""'/></p>"""+
                        """<br /><p>机器人腿间距 　　　   :&nbsp;<input type="text"  style="width:100px; height:30px;" name="w" """+"""value='"""+str(padog.w)+"""'/></p>"""+
                        """<br /><p>步频　　　　:&nbsp;<input type="text"  style="width:100px; height:30px;" name="speed" """+"""value='"""+str(padog.speed)+"""'/></p>"""+
                        """<br /><p>抬腿高度　　　　:&nbsp;<input type="text"  style="width:100px; height:30px;" name="h" """+"""value='"""+str(padog.h)+"""'/></p>"""+
                        """<br /><p>TROT前进重心调整P　:&nbsp;<input type="text"  style="width:100px; height:30px;" name="trot_cg_f" """+"""value='"""+str(padog.trot_cg_f)+"""'/></p>"""+
                        """<br /><p>TROT后退重心调整P　:&nbsp;<input type="text"  style="width:100px; height:30px;" name="trot_cg_b" """+"""value='"""+str(padog.trot_cg_b)+"""'/></p>"""+
                        """<br /><p>TROT转向重心调整P　:&nbsp;<input type="text"  style="width:100px; height:30px;" name="trot_cg_t" """+"""value='"""+str(padog.trot_cg_t)+"""'/></p>""")
                        cl.sendall(    
                        """<br /><p>高度调节P环　　　　　:&nbsp;<input type="text"  style="width:100px; height:30px;" name="Kp_H" """+"""value='"""+str(padog.Kp_H)+"""'/></p>"""+
                        """<br /><p>俯仰姿态P环　　　　　:&nbsp;<input type="text"  style="width:100px; height:30px;" name="pit_Kp_G" """+"""value='"""+str(padog.pit_Kp_G)+"""'/></p>"""+
                        """<br /><p>俯仰姿态D环　　　　　:&nbsp;<input type="text"  style="width:100px; height:30px;" name="pit_Kd_G" """+"""value='"""+str(padog.pit_Kd_G)+"""'/></p>"""+
                        """<br /><p>滚转姿态P环　　　　　:&nbsp;<input type="text"  style="width:100px; height:30px;" name="rol_Kp_G" """+"""value='"""+str(padog.rol_Kp_G)+"""'/></p>"""+
                        """<br /><p>滚转姿态D环　　　　　:&nbsp;<input type="text"  style="width:100px; height:30px;" name="rol_Kd_G" """+"""value='"""+str(padog.rol_Kd_G)+"""'/></p>"""+
                        """<br /><p>平移姿态P环　　      :&nbsp;<input type="text"  style="width:100px; height:30px;" name="tran_mov_kp" """+"""value='"""+str(padog.tran_mov_kp)+"""'/></p>"""+
                        """<br /><p>控制模式(0:网页 1:航模遥控 2:串口):&nbsp;<input type="text"  style="width:100px; height:30px;" name="CC_M" """+"""value='"""+str(padog.CC_M)+"""'/></p>"""+
                        """<br /><input type="Submit" style="width:100px; height:30px;" value="更改控制器参数"  />  """+
                        """</form>
                        <br />*航模遥控器模式下接线：|CH1(左右):32|CH2(前后):26|CH5(开/关自稳):14|CH6(踏步):27|</br>
                        <br />*串口控制模式下接线：|TX:26|RX:27|GND:舵机接口处黑色针脚|</br>
                        </center>
                        </body>
                        </html>
                        """)
            except Exception as e:
                print("Web发送错误:", str(e))
            
            cl.close()
            
        except Exception as e:
            print("Web连接错误:", str(e))
            try:
                cl.close()
            except:
                pass
