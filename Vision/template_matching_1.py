# This work is licensed under the MIT license.
# Copyright (c) 2013-2023 OpenMV LLC. All rights reserved.
# https://github.com/openmv/openmv/blob/master/LICENSE

import time
import sensor
import image
from image import SEARCH_EX
import os

# 获取当前工作路径
#current_path = os.getcwd()
#print("当前工作路径：", current_path)

# 加载模板
template_lm = image.Image("L_M.pgm")
template_ls = image.Image("L_S.pgm")
template_ll = image.Image("L_L.pgm")
template_rm = image.Image("R_M.pgm")
template_rs = image.Image("R_S.pgm")
template_rl = image.Image("R_L.pgm")
template_b1_1 = image.Image("branch1_1.pgm")
template_b2_1 = image.Image("branch2_1.pgm")
template_o1 = image.Image("obstacle1_M.pgm")
template_o2 = image.Image("obstacle2_M.pgm")
template_o3 = image.Image("obstacle3_M.pgm")
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
    ll = find_image(template_ll, gray_img, color_img,0.6)
    if ll:
        print("Find ll!")
        Ans = 0

    lm = find_image(template_lm, gray_img, color_img,0.6)
    if lm:
        print("Find lm!")
        Ans = 0

    ls = find_image(template_ls, gray_img, color_img,0.6)
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

def find_obstacle(gray_img, color_img):
    o1 = find_image(template_o1, gray_img, color_img,0.45)
    if o1:
        print("Find o1!")

    o2 = find_image(template_o2, gray_img, color_img,0.45)
    if o2:
        print("Find o2!")

    o3 = find_image(template_o3, gray_img, color_img,0.45)
    if o3:
        print("Find o3!")

    return o1 or o2 or o3
