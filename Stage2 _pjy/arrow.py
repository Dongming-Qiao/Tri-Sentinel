import sensor, time

frame_counter = 0
DETECTION_CYCLE = 3

arrow_detected = False
arrow_center = (80, 60)

ARROW_THRESHOLD = (12, 78, -55, -22, -9, 50)

sensor.reset()      # 初始化摄像头
sensor.set_hmirror(True)# 镜像（如果视觉模块倒着安装，则开启这个镜像）
sensor.set_vflip(True)   # 翻转（如果视觉模块倒着安装，则开启这个翻转）
sensor.set_pixformat(sensor.RGB565) # 采集格式（彩色图像采集）
sensor.set_framesize(sensor.QQVGA)    # 像素大小 160X120
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

def detect_arrow(img):
    blobs = img.find_blobs([ARROW_THRESHOLD],
                          roi=(0, 0, 160, 120),
                          pixels_threshold=30,
                          area_threshold=20,
                          merge=True)

    arrow_detected = False
    arrow_center = (80,60)
    max_blob = None

    if blobs:
        max_blob = find_max(blobs)
        area_g = max_blob[2] * max_blob[3]
        if area_g > 40:  # 箭头面积阈值
            arrow_detected = True
            arrow_center = (max_blob.cx(), max_blob.cy())

            img.draw_rectangle(max_blob[0:4], color=(0, 0, 255))
            img.draw_cross(max_blob[5], max_blob[6], color=(0, 0, 255))
            img.draw_string(max_blob[5], max_blob[6]-10, "Arrow", color=(0,0,255))

    return arrow_detected, arrow_center

while(True):

    img = sensor.snapshot()
    frame_counter += 1

    detection_mode = frame_counter % DETECTION_CYCLE

    if detection_mode == 0:
        arrow_detected, arrow_center = detect_arrow(img)
        print("Arrow:", arrow_detected, "Arrow_center:", arrow_center)

    if frame_counter >= 1000:
        frame_counter = 0
