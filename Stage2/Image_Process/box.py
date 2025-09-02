import sensor, time

frame_counter = 0
DETECTION_CYCLE = 3

obstacle_detected = False
obstacle_area = 0

OBSATCLE_THRESHOLD = (19, 53, -19, -1, -16, 31)

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
            # 绘制检测结果（调试用）
            img.draw_rectangle(max_blob[0:4], color=(255, 0, 0))
            img.draw_cross(max_blob[5], max_blob[6], color=(255, 0, 0))

    return obstacle_detected

while(True):

    img = sensor.snapshot()
    frame_counter += 1

    detection_mode = frame_counter % DETECTION_CYCLE

    if detection_mode == 0:
        obstacle_detected = detect_obstacle(img)
        print("Obstacle:", obstacle_detected, " Area:", obstacle_area)
        print(img.get_pixel(40, 30))

    if frame_counter >= 1000:
        frame_counter = 0
