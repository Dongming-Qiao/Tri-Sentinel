# This work is licensed under the MIT license.
# Copyright (c) 2013-2023 OpenMV LLC. All rights reserved.
# https://github.com/openmv/openmv/blob/master/LICENSE

import time
import sensor
import image
from image import SEARCH_EX
import os

# 获取当前工作路径
current_path = os.getcwd()
print("当前工作路径：", current_path)

# 重置传感器
sensor.reset()

# 设置为彩色模式，确保帧缓冲区是彩色的
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QQVGA)

# 传感器参数设置
sensor.set_contrast(1)
sensor.set_gainceiling(16)
sensor.set_vflip(True)
sensor.set_hmirror(True)

# 加载模板
template_lm = image.Image("L_M.pgm")
template_ls = image.Image("L_S.pgm")
template_ll = image.Image("L_L.pgm")
template_rm = image.Image("R_M.pgm")
template_rs = image.Image("R_S.pgm")
template_rl = image.Image("R_L.pgm")
template_b1_1 = image.Image("branch1_1.pgm")
template_b2_1 = image.Image("branch2_1.pgm")
clock = time.clock()

def find_image(template, gray_img, color_img, threshold):
    # 在灰度图上执行匹配
    r = gray_img.find_template(
        template, threshold, step=4, search=SEARCH_EX
    )

    if r:
        # 在彩色图像上绘制匹配框
        color_img.draw_rectangle(r, color=(255, 0, 0))  # 红色框
    return r

def find_pattern(gray_img, color_img):
    #0: Left found
    #1: Right found
    #2: Branch found
    #3: Nothing found

    Ans = 3
    ll = find_image(template_ll, gray_img, color_img,0.7)
    if ll:
        print("Find ll!")
        Ans = 0

    lm = find_image(template_lm, gray_img, color_img,0.7)
    if lm:
        print("Find lm!")
        Ans = 0

    ls = find_image(template_ls, gray_img, color_img,0.7)
    if ls:
        print("Find ls!")
        Ans = 0

    rl = find_image(template_rl, gray_img, color_img,0.7)
    if rl:
        print("Find rl!")
        Ans = 1

    rm = find_image(template_rm, gray_img, color_img,0.7)
    if rm:
        print("Find rm!")
        Ans = 1

    rs = find_image(template_rs, gray_img, color_img,0.7)
    if rs:
        print("Find rs!")
        Ans = 1

    b1_1 = find_image(template_b1_1, gray_img, color_img,0.8)
    if b1_1:
        print("Find branch1_1!")
        Ans = 2

    b2_2 = find_image(template_b2_1, gray_img, color_img,0.8)
    if b2_2:
        print("Find b2_2!")
        Ans = 2
    return Ans

# 主循环
while True:
    clock.tick()
    # 获取彩色图像（帧缓冲区为彩色）
    color_img = sensor.snapshot()

    # 修复：使用copy()方法创建副本，再转换为灰度图
    gray_img = color_img.copy()  # 创建彩色图像的副本
    gray_img.to_grayscale()      # 转换为灰度图，不影响原始彩色图像

    # 执行模板匹配
    result = find_pattern(gray_img, color_img)

    # 此时color_img仍然是彩色的，帧缓冲区保持彩色状态
    # print(clock.fps())
