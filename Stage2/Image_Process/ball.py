import sensor, time

frame_counter = 0
DETECTION_CYCLE = 3

ball_detected = False
ball_position = (0, 0)

BALL_THRESHOLD = (19, 61, 30, 82, 12, 59)

sensor.reset()      # 初始化摄像头
sensor.set_hmirror(True)# 镜像（如果视觉模块倒着安装，则开启这个镜像）
sensor.set_pixformat(sensor.RGB565) # 采集格式（彩色图像采集）
sensor.set_framesize(sensor.QVGA)    # 像素大小 80X60
sensor.skip_frames(time = 2000)     # 等待初始化完成
sensor.set_auto_gain(False) # must be turned off for color tracking
sensor.set_auto_whitebal(False) # must be turned off for color tracking

clock = time.clock()

#在色块集中找到面积最大的色块
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

while(True):

    img = sensor.snapshot()
    frame_counter += 1

    detection_mode = frame_counter % DETECTION_CYCLE

    if detection_mode == 0:
        ball_detected = detect_ball_and_goal(img)
        print("Ball:", ball_detected)

    if frame_counter >= 1000:
        frame_counter = 0
