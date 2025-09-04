import sensor, image, time, display
from pyb import Pin,Timer
from pid import PID
from LQ_Module import motor, Enc_AB

import Receive
import template_matching_1

class State:
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

pid_infra = PID(p = 1000.0, i = 0, d = 0, imax = 6000.0)    # 循迹
pid_x = PID(p = 150,i = 0, d = 0,imax = 50)    # 用于控制摄像头一直朝向障碍物

lcd = display.SPIDisplay()      # 初始化显示屏（参数默认-空）
lcd.clear()                     # 清屏

RIGHT_MAX=60
right_count=RIGHT_MAX

LEFT_MAX=80
left_count=LEFT_MAX

FORWARD_MAX=80
forward_count=FORWARD_MAX

L_WAY_MAX=50
l_way=L_WAY_MAX

F_WAY_MAX=25
f_way=F_WAY_MAX

direction_keep=0

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
sensor.set_framesize(sensor.QQVGA)    # 像素大小是160X120
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
OBSATCLE_THRESHOLD = (23, 60, -2, 5, -11, 19)
OBSATCLE_THRESHOLD_1 = (0, 73, -20, 20, 9, 127)


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
                               roi=(0, 0, 160, 120),
                               pixels_threshold=30,
                               area_threshold=20,
                               merge=True)
    max_ball=None
    if ball_blobs:
        max_ball = find_max(ball_blobs)
        if max_ball.roundness() > 0.4:  # 验证圆形度
            ball_detected = True
            ball_position = (max_ball.cx(), max_ball.cy())
            img.draw_rectangle(max_ball.rect(), color=(0, 255, 0))
            img.draw_string(max_ball.x(), max_ball.y()-10, "Ball", color=(0,255,0))

    return ball_detected, max_ball

def detect_obstacle(img):
    global obstacle_detected, obstacle_area

    obstacle_roi = (0, 0, 160, 120)

    blobs = img.find_blobs([OBSATCLE_THRESHOLD_1],
                          roi=obstacle_roi,
                          pixels_threshold=800,
                          area_threshold=100,
                          merge=True)

    obstacle_detected = False
    max_blob = None
    if blobs:
        max_blob = find_max(blobs)
        area_g = max_blob[2] * max_blob[3]
        if area_g > 2500:  # 障碍物面积阈值
            obstacle_detected = True
            obstacle_area = area_g

            img.draw_rectangle(max_blob[0:4], color=(255, 0, 0))
            img.draw_cross(max_blob[5], max_blob[6], color=(255, 0, 0))

    return obstacle_detected, max_blob

def Tracking(increment, direction):
    global speed_L, speed_R, speed_B, Car_V, E_V, output
    #direction=0: follow right, 1: follow left
    if direction==0:
        E_V = sensor_data[0]*2 + sensor_data[1]*0 - sensor_data[2]*2 - sensor_data[3]*0
        print("E_V:",E_V)
        if sensor_data[3]==0:
            #turn right
            Car_V=0
            output=3000
            speed_L=output
            speed_R=output
            speed_B=output
        elif (sensor_data[0] == 1 and sensor_data[1] == 1 and sensor_data[2] == 1 and sensor_data[3]==1 ):
            #turn left
            Car_V=0
            output=-3000
            speed_L=output
            speed_R=output
            speed_B=output
        elif E_V==0:
            if(sensor_data[0] == 1 and sensor_data[1] == 1 and sensor_data[2] == 1):
                Car_V=-3500
            else:
                Car_V =3500
            speed_L = Car_V
            speed_R = -Car_V
            speed_B = 0
        else:
            output=pid_infra.get_pid(E_V,1)

            Car_V=0

            speed_L=output
            speed_R=output
            speed_B=output
    else:
        E_V = sensor_data[0]*0 + sensor_data[1]*2 - sensor_data[2]*0 - sensor_data[3]*2
        print("E_V:",E_V)
        if sensor_data[0]==0:
            #turn left
            Car_V=0
            output=-3000
            speed_L=output
            speed_R=output
            speed_B=output
        elif (sensor_data[0] == 1 and sensor_data[1] == 1 and sensor_data[2] == 1 and sensor_data[3]==1 ):
            #turn right
            Car_V=0
            output=3000
            speed_L=output
            speed_R=output
            speed_B=output
        elif E_V==0:
            if(sensor_data[3] == 1 and sensor_data[1] == 1 and sensor_data[2] == 1):
                Car_V=-3500
            else:
                Car_V =3500
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
    speed_R += increment
    speed_L = max(-15000, min(15000, speed_L))
    speed_R = max(-15000, min(15000, speed_R))
    speed_B = max(-15000, min(15000, speed_B))

    motor1.run(speed_R)
    motor2.run(speed_B)
    motor3.run(speed_L)

    print("speed_[L,R,B]:",speed_L,speed_R,speed_B)
    print("Car_V,output",Car_V,output)

#矩形求交
def rect_intersection(rect1, rect2):
    """
    计算两个矩形的交集

    参数:
        rect1: 第一个矩形，格式为(x, y, w, h)，其中(x,y)是左上角坐标，w是宽度，h是高度
        rect2: 第二个矩形，格式同上

    返回:
        如果有交集，返回交集矩形(x, y, w, h)；否则返回None
    """
    # 解析第一个矩形的参数
    x1=rect1.x()
    y1=rect1.y()
    w1=rect1.w()
    h1=rect1.h()

    # 计算第一个矩形的右下角坐标
    x2_1 = x1 + w1
    y2_1 = y1 + h1

    # 解析第二个矩形的参数
    x1_2, y1_2, w2, h2 = rect2
    # 计算第二个矩形的右下角坐标
    x2_2 = x1_2 + w2
    y2_2 = y1_2 + h2

    # 计算交集矩形的左上角坐标
    inter_x = max(x1, x1_2)
    inter_y = max(y1, y1_2)

    # 计算交集矩形的右下角坐标
    inter_x2 = min(x2_1, x2_2)
    inter_y2 = min(y2_1, y2_2)

    # 判断是否存在交集
    if inter_x < inter_x2 and inter_y < inter_y2:
        # 计算交集矩形的宽度和高度
        inter_w = inter_x2 - inter_x
        inter_h = inter_y2 - inter_y
        return (inter_x, inter_y, inter_w, inter_h)
    else:
        return None


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

    if state!=State.OBSTACLE_AVOIDANCE:
        state = State.LINE_FOLLOWING
        if detection_mode == 0:
            #color_matching
            obstacle_detected_color, max_blob = detect_obstacle(img)
            if max_blob==None or max_blob.density()<0.6:
                obstacle_detected_color=0


            #template_matching
            gray_img = img.copy()     # 创建彩色图像的副本
            gray_img.to_grayscale()   # 转换为灰度图，不影响原始彩色图像
            obstacle_detected_template= template_matching_1.find_obstacle(gray_img, img)

            obstacle_detected=False
            if obstacle_detected_color and obstacle_detected_template:
                inter_area=rect_intersection(max_blob, obstacle_detected_template)
                if inter_area:

                    if (inter_area[2]*inter_area[3])/(obstacle_detected_template[2]*obstacle_detected_template[3])>0.7:
                        obstacle_detected=True
                    if obstacle_detected:
                        # 在彩色图像上绘制匹配框
                        img.draw_rectangle(obstacle_detected_template, color=(255, 0, 255))  # 紫色框

            #change state
            if obstacle_detected:
                state = State.OBSTACLE_AVOIDANCE
            #print("Obstacle:", obstacle_detected, " Area:", obstacle_area)
            #print(img.get_pixel(40, 30))
        elif detection_mode == 1:
            ball_detected, ball_area = detect_ball_and_goal(img)

            if ball_detected:
                state = State.KICK_BALL

            #print("Ball:", ball_detected, " Area:", ball_area)
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

    print("state:",state)

    obstacle_flag = 0
    Receive.Receive_Sensor_Data()   # 接收传感器数据
    sensor_data = Receive.Get_Sensor_Data()  # 获取传感器数据
    print(sensor_data)

    if state == State.LINE_FOLLOWING:
        Tracking(increment=0, direction=direction_keep)
        right_count=RIGHT_MAX
        left_count=LEFT_MAX
        forward_count=FORWARD_MAX
        l_way=L_WAY_MAX
        f_way=F_WAY_MAX
    elif state == State.OBSTACLE_AVOIDANCE:
        get_pattern=(sensor_data[0] == 0 and sensor_data[1] == 0 and sensor_data[2] == 0)or(sensor_data[0] == 0 and sensor_data[1] == 0)
        if right_count>0:
            right_count-=1
            #right
            speed_R=0
            speed_B=-3500
            speed_L=3500
        elif forward_count>0:
            forward_count-=1
            #forward
            speed_R=-3000
            speed_B=0
            speed_L=3000
        elif not get_pattern:
            #left
            speed_R=-3500
            speed_B=3500
            speed_L=0
        else:
            state=State.LINE_FOLLOWING
            speed_R=0
            speed_B=0
            speed_L=0
        motor1.run(speed_R)
        motor2.run(speed_B)
        motor3.run(speed_L)
    elif state == State.KICK_BALL:
        # 找球的函数
        pass
    elif state==State.TURN_LEFT:
        #if l_way>0:
        #    l_way-=1
        #    speed_R=-3500
        #    speed_B=3500
        #    speed_L=0
        #elif f_way>0:
        #    f_way-=1
        #    speed_R=-3500
        #    speed_B=0
        #    speed_L=3500
        #else:
        #    state=State.LINE_FOLLOWING
        #    speed_R=0
        #    speed_B=0
        #    speed_L=0
        #motor1.run(speed_R)
        #motor2.run(speed_B)
        #motor3.run(speed_L)
        Tracking(increment=0, direction=1)
        direction_keep=1
    elif state==State.TURN_RIGHT:
        Tracking(increment=0, direction=0)
        direction_keep=0
    elif state==State.TURN_TO_BRANCH:
        Tracking(increment=0, direction=0)
        direction_keep=0
    else:
        pass

    lcd.write(img)  # 显示屏显示图像
