"""
@Untitled - By: LQ008 - Thu May 23 2024
@文件说明：使用本测试例程之前请先将 ‘LQ_Module.py’ 文件复制存放到模块根目录
        测试母板上电机接口和无刷电调接口
"""
import  pyb,time                              # 导入pyb,时间相关模块
from pyb import Timer,Pin          # 从pyb加载 串口，LED，定时器，Pin

# 导入LQ_Module 模块
from LQ_Module import motor, Enc_AB     # 从LQ_Module文件中导入motor

import Receive_bt

# 霍尔编码器引脚初始化
#B(reverse)
Enc1 = Enc_AB(Timer(12, freq=5), Enc_A="P27", Enc_B="P21")
#L(reverse)
Enc2 = Enc_AB(Timer(13, freq=5), Enc_A="P28", Enc_B="P29")
#R(reverse)
Enc3 = Enc_AB(Timer(14, freq=5), Enc_A="P25", Enc_B="P26")



#初始化pwm 及控制io，针对例如LQ8701电机驱动板
motor1 = motor(timer=4, chl=1, freq=10000, pin_pwm="P7", pin_io="P22")
motor2 = motor(timer=4, chl=2, freq=10000, pin_pwm="P8", pin_io="P23")
motor3 = motor(timer=4, chl=3, freq=10000, pin_pwm="P9", pin_io="P24")

#motor1.run(PWM)             # PWM>0 正转，反之反转，pwm 0~24000
#motor1.run_percent(cnt)     # 方式2，百分比，直接传入占空比  0~100

COUNTER_MAX=2000
counter=COUNTER_MAX

#--------------------------------------------------------------------------

# LQ 无刷电调驱动测试

from LQ_Module import motor_brushless

# 在50Hz 下1~2ms范围,可控范围约在 280000~480000起，1ms至2ms
M1= motor_brushless(2, 1, 50, "P14")  # 参数(timer, chl, freq, pin_pwm)，通道定时器对应关系查手册，不可随意更改
M2= motor_brushless(2, 2, 50, "P15")
M3= motor_brushless(2, 3, 50, "P16")


M1.run(2850*100)        # 输出，较小值，无刷电机可以转起来
M2.run(2900*100)
M3.run(3000*100)
#--------------------------------------------------------------------------

action_stored=""

while True:
    speed_L=0
    speed_R=0
    speed_B=0
    output=4000
    Receive_bt.Receive_Data_bt()
    action_bt=Receive_bt.Get_Action_bt()
    #print(action_bt)
    if (action_bt!="failed" and action_bt!="") or (action_stored!="failed" and action_stored!=""):
        if counter==COUNTER_MAX:
            if action_bt!="failed" and action_bt!="":
                action_stored=action_bt
            else:
                pass
            counter-=1
            print("1")
        elif counter>0:
            counter-=1
            print("2")
        else:
            action_stored=""
            counter=COUNTER_MAX
            print("3")
        print(action_stored)
    else:
        pass

    print("action_stored:"+action_stored)
    print(counter)

    if action_stored=="up":
        speed_L=output
        speed_R=-output
        speed_B=0
    elif action_stored=="left":
        speed_L=0
        speed_R=-output
        speed_B=output
    elif action_stored=="right":
        speed_L=output
        speed_R=0
        speed_B=-output
    elif action_stored=="down":
        speed_L=-output
        speed_R=output
        speed_B=0
    elif action_stored=="OK":
        pass
    else:
        speed_L=0
        speed_R=0
        speed_B=0
        print("No actihon.")

    #电机110%反转 2,3以10%占空比正转 输出
    #R
    motor1.run(speed_R)             # -24000 ~ 24000
    #B
    motor2.run(speed_B)
    #L
    motor3.run(speed_L)
    #print(f"{speed_B, speed_L, speed_R}")

