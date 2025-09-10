import sensor, image, time, display
from pyb import Pin, Timer
from pid import PID
from LQ_Module import motor, Enc_AB
import Receive
import template_matching_1

# ==================== 状态枚举定义 ====================
class State:
    """系统状态枚举类"""
    LINE_FOLLOWING = 1      # 循线行驶
    OBSTACLE_AVOIDANCE = 2  # 避障
    KICK_BALL = 3           # 踢球
    TURN_LEFT = 4           # 左转
    TURN_RIGHT = 5          # 右转
    TURN_TO_BRANCH = 6      # 转向分支
    DASH_TO_GOAL = 7        # 冲向球门
    BACK_TO_LINE_BACK = 8   # 后退回线
    BACK_TO_LINE_RIGHT = 9  # 右转回线

# ==================== 全局变量定义 ====================
# 传感器数据
sensor_data = [0, 0, 0, 0]
# 系统状态
current_state = State.LINE_FOLLOWING

# 编码器和电机控制
encoder_value = 0
E_V = 0           # 误差值
Car_V = 0         # 车体速度
output = 0        # PID输出值
speed_L = 0       # 左轮速度
speed_R = 0       # 右轮速度
speed_B = 0       # 后轮速度

# PID控制器
pid_infra = PID(p=800.0, i=2.0, d=-100, imax=6000.0)  # 循迹PID

# 状态计数器
RIGHT_MAX = 80
right_count = RIGHT_MAX
LEFT_MAX = 80
left_count = LEFT_MAX
FORWARD_MAX = 40
forward_count = FORWARD_MAX
L_WAY_MAX = 50
l_way = L_WAY_MAX
F_WAY_MAX = 25
f_way = F_WAY_MAX
DASH_MAX = 70
dash = DASH_MAX
PUSH_BALL_COUNT_MAX = 150
push_ball_count = PUSH_BALL_COUNT_MAX
BACK_TO_LINE_BACK_COUNT_MAX = 70
back_to_line_back = BACK_TO_LINE_BACK_COUNT_MAX

# 视觉相关变量
direction_keep = 0
green_arrow_found = 0
big_ball_found = 0
allow_to_see_red = 0
has_kicked = 0
flag_stop = 0

# 图像中心点
img_center = (80, 60)

# 颜色阈值
BALL_THRESHOLD = (19, 61, 30, 82, 12, 59)
OBSTACLE_THRESHOLD = (0, 73, -20, 20, 9, 127)
ARROW_THRESHOLD = (32, 79, -83, -23, -9, 57)

# 硬件问题的修正，左右轮速度不同，需要一个常量进行修正
FIX = 300

# ==================== 硬件初始化 ====================
# 按键初始化
button_0 = Pin('P30', Pin.IN, Pin.PULL_UP)

# 电机初始化
# R - 右轮电机
motor1 = motor(timer=4, chl=1, freq=10000, pin_pwm="P7", pin_io="P22")
# B - 后轮电机
motor2 = motor(timer=4, chl=2, freq=10000, pin_pwm="P8", pin_io="P23")
# L - 左轮电机
motor3 = motor(timer=4, chl=3, freq=10000, pin_pwm="P9", pin_io="P24")

# 编码器初始化
# B - 后轮编码器
Enc1 = Enc_AB(Timer(12, freq=5), Enc_A="P27", Enc_B="P21")
# L - 左轮编码器
Enc2 = Enc_AB(Timer(13, freq=5), Enc_A="P28", Enc_B="P29")
# R - 右轮编码器
Enc3 = Enc_AB(Timer(14, freq=5), Enc_A="P25", Enc_B="P26")

# 摄像头初始化
sensor.reset()
sensor.set_hmirror(True)    # 水平镜像
sensor.set_vflip(True)      # 垂直翻转
sensor.set_pixformat(sensor.RGB565)  # 彩色图像
sensor.set_framesize(sensor.QQVGA)   # 160x120分辨率
sensor.skip_frames(time=2000)        # 等待初始化
sensor.set_auto_gain(False)          # 关闭自动增益
sensor.set_auto_whitebal(False)      # 关闭自动白平衡

# 显示初始化
lcd = display.SPIDisplay()
lcd.clear()

# 时钟和帧计数器
clock = time.clock()
frame_counter = 0
DETECTION_CYCLE = 3  # 检测周期

# ==================== 功能函数说明Part1:检测相关函数 ====================
# 检测相关函数

# find_max() - 寻找最大的色块，用于各种物体检测中确定主要目标

# detect_ball_and_goal() - 检测球和球门，通过颜色阈值和圆形度验证识别球体

# detect_obstacle() - 检测障碍物，使用颜色阈值识别前方障碍物

# detect_arrow() - 检测箭头，识别绿色箭头用于导航

# rect_intersection() - 计算两个矩形的交集，用于多传感器数据融合验证

# obstacle_detection_process() - 障碍物检测主流程，结合颜色检测和模板匹配

# 各函数作用位置：
# find_max() - 被各种检测函数调用，用于确定主要目标
# detect_ball_and_goal() - 在KICK_BALL状态和球检测模式中使用
# detect_obstacle() - 在障碍物检测流程中被调用
# detect_arrow() - 在推球和冲向球门状态中使用
# rect_intersection() - 在障碍物检测流程中用于验证检测结果
# obstacle_detection_process() - 在主循环的障碍物检测模式中调用
# =====================================================

# ==================== 功能函数定义Part1:检测相关函数 ====================
def find_max(blobs):
    """寻找最大的色块"""
    max_size = 0
    max_blob = None
    for blob in blobs:
        if blob[2] * blob[3] > max_size:
            max_blob = blob
            max_size = blob[2] * blob[3]
    return max_blob

def detect_ball_and_goal(img):
    """检测球和球门"""
    global ball_detected, ball_position
    
    ball_detected = False
    ball_blobs = img.find_blobs([BALL_THRESHOLD],
                               roi=(0, 0, 160, 120),
                               pixels_threshold=30,
                               area_threshold=20,
                               merge=True)
    
    max_ball = None
    if ball_blobs:
        max_ball = find_max(ball_blobs)
        if max_ball.roundness() > 0.7:  # 验证圆形度
            ball_detected = True
            ball_position = (max_ball.cx(), max_ball.cy())
            img.draw_rectangle(max_ball.rect(), color=(0, 255, 0))
            img.draw_string(max_ball.x(), max_ball.y()-10, "Ball", color=(0, 255, 0))
    
    return ball_detected, max_ball

def detect_obstacle(img):
    """检测障碍物"""
    global obstacle_detected, obstacle_area
    
    obstacle_roi = (0, 0, 160, 120)
    blobs = img.find_blobs([OBSTACLE_THRESHOLD],
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

def detect_arrow(img):
    """检测箭头"""
    blobs = img.find_blobs([ARROW_THRESHOLD],
                          roi=(0, 0, 160, 120),
                          pixels_threshold=17,
                          area_threshold=9,
                          merge=True)
    
    arrow_detected = False
    arrow_center = (80, 60)
    max_blob = None
    
    if blobs:
        max_blob = find_max(blobs)
        area_g = max_blob[2] * max_blob[3]
        if area_g > 25:  # 箭头面积阈值
            arrow_detected = True
            arrow_center = (max_blob.cx(), max_blob.cy())
            img.draw_rectangle(max_blob[0:4], color=(0, 0, 255))
            img.draw_cross(max_blob[5], max_blob[6], color=(0, 0, 255))
            img.draw_string(max_blob[5], max_blob[6]-10, "Arrow", color=(0, 0, 255))
    
    return arrow_detected, arrow_center

def rect_intersection(rect1, rect2):
    """
    计算两个矩形的交集
    
    参数:
        rect1: 第一个矩形，(x, y, w, h)
        rect2: 第二个矩形，(x, y, w, h)
    
    返回:
        交集矩形 (x, y, w, h) 或 None
    """
    # 解析第一个矩形
    x1 = rect1.x()
    y1 = rect1.y()
    w1 = rect1.w()
    h1 = rect1.h()
    x2_1 = x1 + w1
    y2_1 = y1 + h1
    
    # 解析第二个矩形
    x1_2, y1_2, w2, h2 = rect2
    x2_2 = x1_2 + w2
    y2_2 = y1_2 + h2
    
    # 计算交集
    inter_x = max(x1, x1_2)
    inter_y = max(y1, y1_2)
    inter_x2 = min(x2_1, x2_2)
    inter_y2 = min(y2_1, y2_2)
    
    # 检查是否存在交集
    if inter_x < inter_x2 and inter_y < inter_y2:
        inter_w = inter_x2 - inter_x
        inter_h = inter_y2 - inter_y
        return (inter_x, inter_y, inter_w, inter_h)
    else:
        return None

def obstacle_detection_process(img):
    """
    障碍物检测处理流程
    返回: 是否检测到障碍物 (bool)
    """
    # 颜色检测
    obstacle_detected_color, max_blob = detect_obstacle(img)
    if max_blob is None or max_blob.density() < 0.6:
        obstacle_detected_color = False
    
    # 模板匹配检测
    gray_img = img.copy()
    gray_img.to_grayscale()
    obstacle_detected_template = template_matching_1.find_obstacle(gray_img, img)
    
    obstacle_detected = False
    if obstacle_detected_color and obstacle_detected_template:
        inter_area = rect_intersection(max_blob, obstacle_detected_template)
        if inter_area:
            area_ratio = (inter_area[2] * inter_area[3]) / (obstacle_detected_template[2] * obstacle_detected_template[3])
            if area_ratio > 0.7:
                obstacle_detected = True
                img.draw_rectangle(obstacle_detected_template, color=(255, 0, 255))  # 紫色框
    
    return obstacle_detected

# ==================== 功能函数说明 ====================
# 动作执行相关函数：

# push_ball() - 推球功能，控制小车推动球体前进

# Tracking() - 循迹控制函数，根据传感器数据实现PID循迹算法

# obstacle_avoidance_behavior() - 避障状态行为处理，实现右转-前进-左转的避障策略

# check_back_to_line() - 检查是否回到线上，通过传感器数据判断是否完成回线

# back_to_line_right_behavior() - 右转回线状态行为处理，控制小车右转回到循线状态

# 各函数作用位置：
# push_ball() - 在KICK_BALL状态中执行推球动作
# Tracking() - 在LINE_FOLLOWING、TURN_LEFT、TURN_RIGHT、TURN_TO_BRANCH状态中使用
# obstacle_avoidance_behavior() - 在OBSTACLE_AVOIDANCE状态中处理避障行为
# check_back_to_line() - 在回线检查中被back_to_line_right_behavior()调用
# back_to_line_right_behavior() - 在BACK_TO_LINE_RIGHT状态中处理回线行为
# =====================================================

# ==================== 功能函数定义Part2:动作执行相关函数 ====================

def push_ball(img):
    """推球功能"""
    global sensor_data, speed_L, speed_R, speed_B, img_center
    global green_arrow_found, push_ball_count, big_ball_found, Car_V
    
    modify_speed = 0
    arrow_detected = False
    cruise = False
    
    # 检查是否在巡航状态
    if sensor_data[0] != 1 or sensor_data[1] != 1 or sensor_data[2] != 1:
        cruise = False
        if green_arrow_found == 0:
            ball_detected, ball_area = detect_ball_and_goal(img)
            if (ball_area is not None and ball_area.area() > 400 and 
                40 < ball_area.x() < 120) or big_ball_found:
                print("BIG BALL")
                ball_center = (ball_area.cx(), ball_area.cy())
                ball_error_x = ball_center[0] - img_center[0]
                motor1.run(-3700 + 15 * ball_error_x)
                motor2.run(0)
                motor3.run(3500 + 15 * ball_error_x)
                big_ball_found = 1
            else:
                Tracking(0, 0, 1)  # 循迹
    else:
        cruise = True
        arrow_detected, arrow_center = detect_arrow(img)
        Car_V = 3500
        
        if arrow_detected:
            # 根据箭头位置调整速度
            Bias_x = -img_center[0] + arrow_center[0]
            Bias_y = -img_center[1] + arrow_center[1]
            print("Arrow Bias:", Bias_x, Bias_y)
            
            modify_speed = Bias_x * 15
            speed_L = Car_V + modify_speed
            speed_R = -Car_V + modify_speed - FIX
            speed_B = modify_speed * 1.15
        else:
            # 未检测到箭头，继续前进
            speed_L = Car_V
            speed_R = -Car_V - FIX
            speed_B = 0
            
            # 速度限制
            if 0 < speed_L < 3000:
                speed_L = 3000
            if -3000 < speed_L < 0:
                speed_L = -3000
            if 0 < speed_R < 3000:
                speed_R = 3000
            if -3000 < speed_R < 0:
                speed_R = -3000
        
        motor1.run(speed_R)
        motor2.run(speed_B)
        motor3.run(speed_L)
        print("speed_[L,R,B]:", speed_L, speed_R, speed_B)
    
    print("cruise", cruise)
    return arrow_detected

def Tracking(increment, direction, pushing_ball):
    """循迹控制函数"""
    global speed_L, speed_R, speed_B, Car_V, E_V, output
    
    # 速度参数定义
    straight_speed = 3500
    rotate_speed = 3000
    
    if direction == 0:  # 右侧循迹
        E_V = sensor_data[0]*2 + sensor_data[1]*0 - sensor_data[2]*2 - sensor_data[3]*0
        print("E_V:", E_V)
        
        if sensor_data[3] == 0:  # 需要右转
            Car_V = 0
            output = rotate_speed
            speed_L = output
            speed_R = output
            speed_B = output
        elif sensor_data[0] == 1 and sensor_data[1] == 1 and sensor_data[2] == 1 and sensor_data[3] == 1:  # 需要左转
            Car_V = 0
            output = -rotate_speed
            speed_L = output
            speed_R = output
            speed_B = output
        elif E_V == 0:  # 直行
            if sensor_data[0] == 1 and sensor_data[1] == 1 and sensor_data[2] == 1:
                Car_V = straight_speed
            else:
                Car_V = straight_speed
            speed_L = Car_V
            speed_R = -Car_V
            speed_B = 0
        else:  # PID控制
            output = pid_infra.get_pid(E_V, 1)
            Car_V = 0
            output = max(-6000, min(6000, output))
            
            # 输出限制
            if 0 < output < 2500:
                output = 2500
            if -2500 < output < 0:
                output = -2500
                
            speed_L = output
            speed_R = output
            speed_B = output
    else:  # 左侧循迹
        E_V = sensor_data[0]*0 + sensor_data[1]*2 - sensor_data[2]*0 - sensor_data[3]*2
        print("E_V:", E_V)
        
        if sensor_data[0] == 0:  # 需要左转
            Car_V = 0
            output = rotate_speed
            speed_L = output
            speed_R = output
            speed_B = output
        elif sensor_data[0] == 1 and sensor_data[1] == 1 and sensor_data[2] == 1 and sensor_data[3] == 1:  # 需要右转
            Car_V = 0
            output = rotate_speed
            speed_L = output
            speed_R = output
            speed_B = output
        elif E_V == 0:  # 直行
            if sensor_data[3] == 1 and sensor_data[1] == 1 and sensor_data[2] == 1:
                Car_V = straight_speed
            else:
                Car_V = straight_speed
            speed_L = Car_V
            speed_R = -Car_V
            speed_B = 0
        else:  # PID控制
            output = pid_infra.get_pid(E_V, 1)
            Car_V = 0
            output = max(-6000, min(6000, output))
            
            # 输出限制
            if 0 < output < 2500:
                output = 2500
            if -2500 < output < 0:
                output = -2500
                
            speed_L = output
            speed_R = output
            speed_B = output
    
    # 应用增量并限制速度范围
    speed_L += increment
    speed_R += increment
    speed_L = max(-15000, min(15000, speed_L))
    speed_R = max(-15000, min(15000, speed_R))
    speed_B = max(-15000, min(15000, speed_B))
    
    # 执行电机控制
    motor1.run(speed_R)
    motor2.run(speed_B)
    motor3.run(speed_L)
    
    print("speed_[L,R,B]:", speed_L, speed_R, speed_B)
    print("Car_V,output", Car_V, output)

def obstacle_avoidance_behavior(sensor_data):
    """
    避障状态行为处理
    返回: 下一个状态, 右轮速度, 后轮速度, 左轮速度
    """
    get_pattern = (sensor_data[0] == 0 and sensor_data[1] == 0 and sensor_data[2] == 0) or (sensor_data[0] == 0 and sensor_data[1] == 0)
    
    if right_count > 0:  # 右转阶段
        right_count -= 1
        return State.OBSTACLE_AVOIDANCE, 0, -3500, 3420
    elif forward_count > 0:  # 前进阶段
        forward_count -= 1
        return State.OBSTACLE_AVOIDANCE, -3500, 0, 3000
    elif not get_pattern:  # 左转阶段
        return State.OBSTACLE_AVOIDANCE, -3550, 3500, 0
    else:  # 返回循线
        return State.LINE_FOLLOWING, 0, 0, 0
    
def check_back_to_line(sensor_data):
    """
    检查是否回到线上
    返回: 是否回到线上的布尔值
    """
    get_white_0000 = (sensor_data[0] == 0 and sensor_data[1] == 0 and sensor_data[2] == 0 and sensor_data[3] == 0)
    get_white_00xx = (sensor_data[0] == 0 and sensor_data[1] == 0)
    get_white_xx00 = (sensor_data[2] == 0 and sensor_data[3] == 0)
    get_white_000x = (sensor_data[0] == 0 and sensor_data[1] == 0 and sensor_data[2] == 0)
    get_white_x000 = (sensor_data[1] == 0 and sensor_data[2] == 0 and sensor_data[3] == 0)
    
    return get_white_0000 or get_white_00xx or get_white_xx00 or get_white_000x or get_white_x000

def back_to_line_right_behavior(sensor_data):
    """
    右转回线状态行为处理
    返回: 下一个状态, 右轮速度, 后轮速度, 左轮速度
    """
    direction_keep = 0
    
    if not check_back_to_line(sensor_data):  # 继续右转
        return State.BACK_TO_LINE_RIGHT, -3550, 0, 3850
    else:  # 返回循线
        return State.LINE_FOLLOWING, 0, 0, 0

# ==================== 主循环 ====================
while True:
    print(green_arrow_found)
    
    # 按键检测
    if not button_0.value():  # 检测K0按键按下
        while not button_0.value():  # 等待按键松开
            pass
        start_flag = not start_flag  # 切换启动标志
    
    # 获取图像
    img = sensor.snapshot()
    # 每DETECTION_CYCLE个帧一个周期，一个周期里面的每个帧各检测关键物体，保证检测频率的同时减少计算量
    frame_counter += 1
    detection_mode = frame_counter % DETECTION_CYCLE
    
    if frame_counter >= 1000:
        frame_counter = 0
    
    # 状态检测与转换
    if (current_state != State.OBSTACLE_AVOIDANCE and 
        current_state != State.KICK_BALL and 
        current_state != State.DASH_TO_GOAL and 
        current_state != State.BACK_TO_LINE_BACK and 
        current_state != State.BACK_TO_LINE_RIGHT):
        
        current_state = State.LINE_FOLLOWING
        
        if detection_mode == 0:  # 障碍物检测
            # 检测函数
            obstacle_detected = obstacle_detection_process(img)
            
            # 状态转换
            if obstacle_detected:
                current_state = State.OBSTACLE_AVOIDANCE
                
        elif detection_mode == 1:  # 球检测
            # 检测函数
            ball_detected, ball_area = detect_ball_and_goal(img)
            
            # 状态转换
            if allow_to_see_red and ball_detected and not has_kicked:
                current_state = State.KICK_BALL
                has_kicked = 1
                
        else:  # 路径检测
            # 检测程序，模板匹配
            gray_img = img.copy()
            gray_img.to_grayscale()
            template_matching_index = template_matching_1.find_pattern(gray_img, img)
            
            # 模板匹配结果处理
            if template_matching_index == 0:  # 左转标志
                current_state = State.TURN_LEFT
            elif template_matching_index == 1:  # 右转标志
                current_state = State.TURN_RIGHT
            elif template_matching_index == 2:  # 分支标志
                current_state = State.TURN_TO_BRANCH
    
    print("state:", current_state)
    
    # 获取传感器数据
    Receive.Receive_Sensor_Data()
    sensor_data = Receive.Get_Sensor_Data()
    print(sensor_data)
    
    # 状态处理
    if current_state == State.LINE_FOLLOWING:
        # 循线行驶状态
        Tracking(increment=0, direction=direction_keep, pushing_ball=0)

        # 刷新变量值
        right_count = RIGHT_MAX
        left_count = LEFT_MAX
        forward_count = FORWARD_MAX
        l_way = L_WAY_MAX
        f_way = F_WAY_MAX
        push_ball_count = PUSH_BALL_COUNT_MAX

        green_arrow_found = 0
        
    elif current_state == State.OBSTACLE_AVOIDANCE:
        # 避障状态
        next_state, speed_R, speed_B, speed_L = obstacle_avoidance_behavior(sensor_data)
        current_state = next_state

        motor1.run(speed_R)
        motor2.run(speed_B)
        motor3.run(speed_L)

        
    elif current_state == State.KICK_BALL:
        # 踢球状态
        detect_ball_and_goal(img)  # 找球
        arrow_detected_temp = push_ball(img)
            
        if green_arrow_found == 0:
            green_arrow_found = arrow_detected_temp
                
        if green_arrow_found:  # 检查是否到终点
            get_end_0000 = (sensor_data[0] == 0 and sensor_data[1] == 0 and sensor_data[2] == 0 and sensor_data[3] == 0)
            get_end_x000 = (sensor_data[1] == 0 and sensor_data[2] == 0 and sensor_data[3] == 0)
            get_end_000x = (sensor_data[0] == 0 and sensor_data[1] == 0 and sensor_data[2] == 0)
            get_end = get_end_0000 or get_end_x000 or get_end_000x
                
            if get_end:  # 到达终点，停止
                speed_R = 0
                speed_B = 0
                speed_L = 0
                motor1.run(speed_R)
                motor2.run(speed_B)
                motor3.run(speed_L)
                current_state = State.DASH_TO_GOAL
                dash = DASH_MAX
            
    elif current_state == State.DASH_TO_GOAL:
        # 冲向球门状态
        if dash > 0:
            arrow_detected, arrow_center = detect_arrow(img)
            Car_V = 3500
            
            if arrow_detected:  # 根据箭头调整
                Bias_x = -img_center[0] + arrow_center[0]
                Bias_y = -img_center[1] + arrow_center[1]
                print("Arrow Bias:", Bias_x, Bias_y)
                
                modify_speed = Bias_x * 20
                speed_L = Car_V + modify_speed
                speed_R = -Car_V + modify_speed - FIX
                speed_B = modify_speed * 1.15
            else:  # 直行
                speed_L = Car_V
                speed_R = -Car_V
                speed_B = 0
                
            dash -= 1
        else:  # 切换状态
            current_state = State.BACK_TO_LINE_BACK
            speed_R = 0
            speed_B = 0
            speed_L = 0
            back_to_line_back_count = BACK_TO_LINE_BACK_COUNT_MAX
            
        motor1.run(speed_R)
        motor2.run(speed_B)
        motor3.run(speed_L)
        
    elif current_state == State.BACK_TO_LINE_BACK:
        # 后退回线状态
        if back_to_line_back_count > 0:
            back_to_line_back_count -= 1
            motor1.run(3500 + FIX)
            motor2.run(0)
            motor3.run(-3500)
        else:
            current_state = State.BACK_TO_LINE_RIGHT
            
    elif current_state == State.BACK_TO_LINE_RIGHT:
        # 右转回线状态 - 使用封装的行为函数
        next_state, speed_R, speed_B, speed_L = back_to_line_right_behavior(sensor_data)
        current_state = next_state
    
        motor1.run(speed_R)
        motor2.run(speed_B)
        motor3.run(speed_L)
        
    else:
        # 未知状态处理
        pass
    
    # 显示图像
    lcd.write(img)