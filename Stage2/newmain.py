from enum import Enum

import sensor, image, time, display     
from pyb import Pin,Timer              
from pid import PID                    
from LQ_Module import motor, Enc_AB 

import Receive
import template_matching_1

class State(Enum):
    LINE_FOLLOWING = 1
    OBSTACLE_AVOIDANCE = 2
    CHOOSE_PATH = 3
    KICK_BALL = 4
    TURN_LEFT = 5
    TURN_RIGHT = 6
    TURN_TO_BRANCH = 7

sensor_data = [0, 0, 0, 0]

start_flag = False

state = State.LINE_FOLLOWING

encoder_value = 0       
min_speed = 2000       
speed = 4000 

E_V = 0 # 误差值
Car_V = 0   # 车体速度
output = 0  # PID输出值

speed_L = 0             # 左轮速度暂存全局变量
speed_R = 0             # 右轮速度暂存全局变量
speed_B = 0             # 后轮速度暂存全局变量

pid_infra = PID(p = 1500.0, i = 0, d = 0, imax = 2000.0)    # 循迹
pid_x = PID(p = 150,i = 0, d = 0,imax = 50)    # 用于控制摄像头一直朝向障碍物

lcd = display.SPIDisplay()      # 初始化显示屏（参数默认-空）
lcd.clear()                     # 清屏
pic = image.Image("/pic0.jpg")  # 读取图片
lcd.write(pic) 

#按键初始化,按键扫描，母版上K0,K1,K2分别对应P30,P31,P1
button_0 = Pin('P30', Pin.IN, Pin.PULL_UP)
#button_1 = Pin('P31', Pin.IN, Pin.PULL_UP)

# 初始化三路电机控制PWM及DIR
#R
motor1 = motor(timer=4, chl=1, freq=10000, pin_pwm="P7", pin_io="P22")
#B
motor2 = motor(timer=4, chl=2, freq=10000, pin_pwm="P8", pin_io="P23")
#L
motor3 = motor(timer=4, chl=3, freq=10000, pin_pwm="P9", pin_io="P24")

# 霍尔编码器引脚初始化
#B
Enc1 = Enc_AB(Timer(12, freq=5), Enc_A="P27", Enc_B="P21")
#L
Enc2 = Enc_AB(Timer(13, freq=5), Enc_A="P28", Enc_B="P29")
#R
Enc3 = Enc_AB(Timer(14, freq=5), Enc_A="P25", Enc_B="P26")

sensor.reset()      # 初始化摄像头
sensor.set_hmirror(True)# 镜像（如果视觉模块倒着安装，则开启这个镜像）
sensor.set_vflip(True)   # 翻转（如果视觉模块倒着安装，则开启这个翻转）
sensor.set_pixformat(sensor.RGB565) # 采集格式（彩色图像采集）
sensor.set_framesize(sensor.QQVGA)    # 像素大小不是320X240
#需要修改roi
sensor.skip_frames(time = 2000)     # 等待初始化完成
sensor.set_auto_gain(False) # must be turned off for color tracking
sensor.set_auto_whitebal(False) # must be turned off for color tracking

clock = time.clock()

frame_counter = 0
DETECTION_CYCLE = 3

obstacle_detected = False
obstacle_area = 0

ball_detected = False
ball_position = (0, 0)

BALL_THRESHOLD = (19, 61, 30, 82, 12, 59)
OBSATCLE_THRESHOLD = (19, 53, -19, -1, -16, 31)

def find_max(blobs):
    max_size=0
    for blob in blobs:
        if blob[2]*blob[3] > max_size:
            max_blob=blob
            max_size = blob[2]*blob[3]
    return max_blob

def detect_ball_and_goal(img):
    global ball_detected, ball_position

    ball_detected = False

    ball_blobs = img.find_blobs([BALL_THRESHOLD],
                               roi=(0, 0, 320, 240),
                               pixels_threshold=30,
                               area_threshold=20,
                               merge=True)

    if ball_blobs:
        max_ball = find_max(ball_blobs)
        if max_ball.roundness() > 0.4:  # 验证圆形度
            ball_detected = True
            ball_position = (max_ball.cx(), max_ball.cy())
            img.draw_rectangle(max_ball.rect(), color=(0, 255, 0))
            img.draw_string(max_ball.x(), max_ball.y()-10, "Ball", color=(0,255,0))

    return ball_detected

def detect_obstacle(img):
    global obstacle_detected, obstacle_area

    obstacle_roi = (0, 0, 320, 240)

    blobs = img.find_blobs([OBSATCLE_THRESHOLD],
                          roi=obstacle_roi,
                          pixels_threshold=100,
                          area_threshold=50,
                          merge=True)

    obstacle_detected = False
    if blobs:
        max_blob = find_max(blobs)
        area_g = max_blob[2] * max_blob[3]
        if area_g > 1500:  # 障碍物面积阈值
            obstacle_detected = True
            obstacle_area = area_g
            """ # 绘制检测结果（调试用）
            img.draw_rectangle(max_blob[0:4], color=(255, 0, 0))
            img.draw_cross(max_blob[5], max_blob[6], color=(255, 0, 0)) """

    return obstacle_detected, max_blob

def Tracking():
    global speed_L, speed_R, speed_B, Car_V, E_V, output

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
    
    speed_L += increment
    speed_L += increment
    speed_L = max(-8000, min(8000, speed_L))    
    speed_R = max(-8000, min(8000, speed_R))  
    speed_B = max(-8000, min(8000, speed_B)) 
    
    motor1.run(speed_R)
    motor2.run(speed_B)
    motor3.run(speed_L)



while(True):

     #按键K0切换电机转动标志位
    if not button_0.value():                # 如果检测到K0按键按下
        while not button_0.value():         # 等待按键松开
            pass
        start_flag = not(start_flag)        # 按键松开后取反start_flag的值，控制电机启停

    img = sensor.snapshot()                 # 获取一帧图像

    frame_counter += 1
    detection_mode = frame_counter % DETECTION_CYCLE

    if frame_counter >= 1000:
        frame_counter = 0

    state = State.LINE_FOLLOWING
    if detection_mode == 0:
        obstacle_detected = detect_obstacle(img)

        if obstacle_detected:
            state = State.OBSTACLE_AVOIDANCE

        print("Obstacle:", obstacle_detected, " Area:", obstacle_area)
        print(img.get_pixel(40, 30))
    elif detection_mode == 1:
        ball_detected, max_blob = detect_ball_and_goal(img)

        if ball_detected:
            state = State.KICK_BALL

        print("Ball:", ball_detected)
    else:
        # 修复：使用copy()方法创建副本，再转换为灰度图
        gray_img = img.copy()     # 创建彩色图像的副本
        gray_img.to_grayscale()   # 转换为灰度图，不影响原始彩色图像
        template_matching_index = template_matching_1.find_pattern(gray_img, img)
        #template_matching_index
        #0: Left found
        #1: Right found
        #2: Branch found
        #3: Nothing found
        if template_matching_index==0:
            state = State.TURN_LEFT
        elif template_matching_index==1:
            state = State.TURN_RIGHT
        elif template_matching_index==2:
            state = State.TURN_TO_BRANCH
        else:
            pass

    obstacle_flag = 0
    Receive.Receive_Sensor_Data()   # 接收传感器数据
    sensor_data = Receive.Get_Sensor_Data()  # 获取传感器数据

    E_V = sensor_data[0]*2 + sensor_data[1]*1.2 - sensor_data[2]*1.2 - sensor_data[3]*2

    if state == State.LINE_FOLLOWING:
        Tracking(0)
    elif state == State.OBSTACLE_AVOIDANCE:
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
    elif state == State.KICK_BALL:
        # 找球的函数
        pass
    elif state==State.TURN_LEFT:
        Tracking(500)
    elif state==State.TURN_RIGHT:
        Tracking(-500)
    elif state==State.TURN_TO_BRANCH:
        Tracking(-500)
    else:
        pass

    lcd.write(img)  # 显示屏显示图像