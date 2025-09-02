"""
--- 巡线避障中心停车任务三参考程序，为能快速理解程序，特别写了较为详细的注释
"""
# --------------------------------导入外部文件中的包和模块减↓↓↓----------------------------
import sensor, image, time, display     # 导入摄像头传感器，图像，时间，显示器相关包
from pyb import Pin,Timer               # 从pyb包中导入Pin，Timer模块
from pid import PID                     # 导入PID
from LQ_Module import motor, Enc_AB     # 从LQ_Module文件中导入motor

import Receive                         # 导入Receive模块，用于串口接收数据
# ================================== 主程序 ======================================

# --------------------------------设置初始变量、参数↓↓↓--------------------------------------
#障碍物阈值，纸盒子的褐色的阈值
obstacle_threshold = [
    (24, 71, 28, 87, 0, 55),      #R
    #(38, 63, -62, -41, 33, 61),    #G
    (22, 46, -52, -34, 31, 51)
]

# 传感器数据
sensor_data = [0, 0, 0, 0]  # 四个传感器数据暂存数组

size_threshold = 7250   # 障碍物大小为此值时切换成避障模式
obstacle_flag = 0       # 障碍物标志位，是否识别到障碍物
start_flag = False      # 电机转动标志位，通过K0按键切换，为True时电机转动，否则电机不转

encoder_value = 0       # 编码器累计值(用于记录后轮编码器转动距离)
min_speed = 2000        # 最小速度（电机的死区）
speed = 4000            # 目标速度（控制整体前进速度）

E_V = 0 # 误差值
Car_V = 0   # 车体速度
output = 0  # PID输出值

speed_L = 0             # 左轮速度暂存全局变量（各电机的实际速度值：基准±巡线偏差值）
speed_R = 0             # 右轮速度暂存全局变量
speed_B = 0             # 后轮速度暂存全局变量

""" # PID 参数
pid_x = PID(p = 150,i = 0, d = 0,imax = 50)    # 用于控制摄像头一直朝向障碍物 """

# 红外循迹PID参数
pid_infra = PID(p = 1500.0, i = 0, d = 0, imax = 2000.0)    # 循迹

# ---------------------------TFT-LCD显示初始化↓↓↓--------------------------------------

lcd = display.SPIDisplay()      # 初始化显示屏（参数默认-空）
lcd.clear()                     # 清屏
pic = image.Image("/pic0.jpg")  # 读取图片
lcd.write(pic)                  # 显示图片

# ------------------------------按键初始化↓↓↓--------------------------------------
#按键初始化,按键扫描，母版上K0,K1,K2分别对应P30,P31,P1
button_0 = Pin('P30', Pin.IN, Pin.PULL_UP)
#button_1 = Pin('P31', Pin.IN, Pin.PULL_UP)

# -----------------------------初始化三路电机控制PWM及DIR↓↓↓-------------------------------------
# 电机引脚初始化
motor1 = motor(timer=4, chl=1, freq=10000, pin_pwm="P7", pin_io="P22")
motor2 = motor(timer=4, chl=2, freq=10000, pin_pwm="P8", pin_io="P23")
motor3 = motor(timer=4, chl=3, freq=10000, pin_pwm="P9", pin_io="P24")


# --------------------------初始化霍尔编码器引脚↓↓↓-----------------------------------
# 霍尔编码器引脚初始化
Enc1 = Enc_AB(Timer(12, freq=5), Enc_A="P27", Enc_B="P21")
Enc2 = Enc_AB(Timer(13, freq=5), Enc_A="P28", Enc_B="P29")
Enc3 = Enc_AB(Timer(14, freq=5), Enc_A="P25", Enc_B="P26")

# -----------------------------初始化摄像头↓↓↓-------------------------------------
sensor.reset()      # 初始化摄像头
sensor.set_hmirror(True)# 镜像（如果视觉模块倒着安装，则开启这个镜像）
sensor.set_pixformat(sensor.RGB565) # 采集格式（彩色图像采集）
sensor.set_framesize(sensor.LCD)    # 像素大小 80X60
sensor.skip_frames(time = 2000)     # 等待初始化完成
sensor.set_auto_gain(False) # must be turned off for color tracking
sensor.set_auto_whitebal(False) # must be turned off for color tracking

# -----------------------------自定义函数↓↓↓-------------------------------------
#在色块集中找到面积最大的色块
def find_max(blobs):
    max_size=0
    for blob in blobs:
        if blob[2]*blob[3] > max_size:
            max_blob=blob
            max_size = blob[2]*blob[3]
    return max_blob

# ================================== 主循环 ======================================
while(True):
    #按键K0切换电机转动标志位
    if not button_0.value():                # 如果检测到K0按键按下
        while not button_0.value():         # 等待按键松开
            pass
        start_flag = not(start_flag)        # 按键松开后取反start_flag的值，控制电机启停

    img = sensor.snapshot()                 # 获取一帧图像


    # 寻找障碍,roi = [6, 4, 164, 75]
    blobs = img.find_blobs(obstacle_threshold,pixels_threshold=500, area_threshold=50, merge=True)
    if blobs:   # 如果有阈值中的色块则执行if下面的代码
        max_blob = find_max(blobs)          # 筛选所有色块中的最大的那一个
        area_g = max_blob[2]*max_blob[3]    # 计算障碍物的像素面积
        #print(area_g)                      # 打印当前障碍物的大小
        if area_g > size_threshold:         # 如果障碍物面积超过一定大小，则判断为有障碍物，切换标志位
            obstacle_flag = 1
    else:
        obstacle_flag = 0

    #Test
    obstacle_flag = 0
    Receive.Receive_Sensor_Data()   # 接收传感器数据
    sensor_data = Receive.Get_Sensor_Data()  # 获取传感器数据

    #输出传感器四个数字
    print("a=",sensor_data[0]," b=",sensor_data[1]," c=",sensor_data[2]," d=",sensor_data[3])
    
    E_V = sensor_data[0]*2 + sensor_data[1]*1.2 - sensor_data[2]*1.2 - sensor_data[3]*2

    if obstacle_flag == 0:
        if E_V==0:
            if(sensor_data[0] == 0 & sensor_data[1] == 0 & sensor_data[2] == 0 & sensor_data[3] == 0):
                Car_V=-3000
            else:
                Car_V = 4000
            speed_L = Car_V
            speed_R = -Car_V
            speed_B = 0
        else:
            output=pid_infra.get_pid(E_V,1)
            Car_V=0

            speed_L=output
            speed_R=output
            speed_B=output
        
        speed_L = max(-6000, min(6000, speed_L))    
        speed_R = max(-6000, min(6000, speed_R))  
        speed_B = max(-6000, min(6000, speed_B)) 
    
        motor1.run(speed_L)
        motor2.run(speed_R)
        motor3.run(speed_B)
    

    """ # 切换模式 obstacle_flag=1为避障模式 obstacle_flag=0为寻迹模式
    if obstacle_flag == 1:                      # 执行避障程序
        # 避障思路：识别到障碍物后小车绕着障碍物走（镜头对准障碍物），此时对后轮编码器进行累计，达到一定值（实际测量编码器的值）后
        # 判断为绕障碍物走了180°，此时将车模以一定速度旋转一定角度（掉头），然后将标志位切换回循迹模式。
        img.draw_rectangle(max_blob[0:4])       # 画一个矩形，框出障碍
        img.draw_cross(max_blob[5], max_blob[6])# 障碍中间画一个十字
        area_g = max_blob[2] * max_blob[3]      # 再次计算障碍标识的面积
        error_x = 70 - max_blob[5]              # 障碍中心点与中间的偏差，目的是使镜头一直对着障碍物,以控制障碍物与车的距离，配合后面的编码器累计值完成绕行
        duty_x = pid_x.get_pid(error_x,1)       # pid运算
        #print(duty_x)
        speed_L =  -duty_x              # PID计算出来的值交给电机速度变量如果加距离控制则：-duty_s
        speed_R =  -duty_x              # PID计算出来的值交给电机速度变量如果加距离控制则：+duty_s
        motor1.run(speed_L)             # 左电机
        motor2.run(speed_R)             # 右电机
        motor3.run(4950)                # 后电机  绕障碍旋转，如果转不动，可以增大这个值
        encoder_value += Enc3.Get()     # 编码器开始累计值，累计到一定值后进行旋转180°继续循迹黑线

        if encoder_value > 3050 or encoder_value < -3050:   # 到达预定位置，若error_x， 后的数字越大表示车离障碍物中心越远，想要到达预定位置就需要走更远，编码器的预定值就需要增大
            motor1.run(2500)            # 左电机
            motor2.run(2500)            # 右电机
            motor3.run(2500)            # 后电机
            time.sleep_ms(1500)
            obstacle_flag = 0           # 清除标志位，切换到正常寻迹模式
            encoder_value = 0           # 编码器累计值清零
        lcd.write(img)                  # 显示屏显示图像
        continue    #跳出本次循环
    else:
        #使用img.find_blobs()函数获取图像中的各个色块，将获取到的色块对象保存到blobs
        blobs = img.find_blobs(line_threshold, roi = [5, 7, 121, 73],pixels_threshold=10, area_threshold=10, merge=True)
        if blobs:                        # 找到追踪目标
            blob = find_max(blobs)       # 提取blobs中面积最大的一个黑色色块blob
            img.draw_rectangle(blob.rect(),color=(255, 0, 0))       # 根据色块blob位置画红色框
            img.draw_cross(blob.cx(), blob.cy(),color=(0, 0, 255))  # 根据色块位置在中心画蓝色十字
            x_error = blob.cx()-img.width()/2                       # 计算黑色中心偏差x_error

            speed_L = speed + x_error*45            # 控制电机转速进行循迹
            speed_R = -speed + x_error*45

            if x_error>8:
                speed_B = min_speed + x_error*20    # 控制后轮电机转速协助转弯
            elif x_error<-8:
                speed_B = -min_speed + x_error*20   # 控制后轮电机转速协助转弯
            else:
                speed_B = 0
            print(x_error, speed_L,speed_R,speed_B) # 串行终端打印
            if start_flag:          # 标志位为True时电机转动
                motor1.run(speed_L) # 左电机
                motor2.run(speed_R) # 右电机
                motor3.run(speed_B) # 后电机
            else :                  # 否则电机不转
                motor1.run(0)
                motor2.run(0)
                motor3.run(0)
        else:                           # 没有找到目标，停下不动
            blobs = img.find_blobs(Stop_threshold, roi = [1, 0, 127, 83],pixels_threshold=5, area_threshold=10, merge=True)
            if blobs:                   # 找到追踪目标
                blob = find_max(blobs)  # 提取blobs中面积最大的一个黑色色块blob
                img.draw_rectangle(blob.rect(),color=(0, 255, 0))           # 根据色块blob位置画绿色框
                img.draw_cross(blob.cx(), blob.cy(),color=(255, 255, 0))    # 根据色块位置在中心画黄十字
                #start_flag = 0         # 关闭电机输出
                lcd.write(pic)          # 显示图片
                time.sleep_ms(200)      # 延时后停车，在此出调节最终停车的位置是否准确，如果右的车停车前左右偏移，可加入校正（发挥部分）
                motor1.run(0) # 左电机
                motor2.run(0) # 右电机
                motor3.run(0) # 后电机
                while(1):
                    pass        # 完成所有任务后，进入死循环等待重启
            motor1.run(0)
            motor2.run(0)
            motor3.run(0)
    lcd.write(img)  # 显示屏显示图像 """

