# This work is licensed under the MIT license.
# Copyright (c) 2013-2023 OpenMV LLC. All rights reserved.
# https://github.com/openmv/openmv/blob/master/LICENSE

import time
import sensor
import image
from image import SEARCH_EX
import os

##############################################################################
# 图像模板匹配程序
# 功能：识别道路标志、分支和障碍物
##############################################################################

# 加载所有模板图像
template_lm = image.Image("L_M.pgm")      # 左转标志 - 中尺寸
template_ls = image.Image("L_S.pgm")      # 左转标志 - 小尺寸
template_ll = image.Image("L_L.pgm")      # 左转标志 - 大尺寸
template_rm = image.Image("R_M.pgm")      # 右转标志 - 中尺寸
template_rs = image.Image("R_S.pgm")      # 右转标志 - 小尺寸
template_rl = image.Image("R_L.pgm")      # 右转标志 - 大尺寸
template_b1_1 = image.Image("branch1_1.pgm")  # 分支标志 1
template_b2_1 = image.Image("branch2_1.pgm")  # 分支标志 2
template_o1 = image.Image("obstacle1_M.pgm")  # 障碍物 1
template_o2 = image.Image("obstacle2_M.pgm")  # 障碍物 2
template_o3 = image.Image("obstacle3_M.pgm")  # 障碍物 3

clock = time.clock()

##############################################################################
# 函数名：find_image
# 功能：在图像中查找指定模板
# 参数：
#   template: 模板图像
#   gray_img: 灰度输入图像
#   color_img: 彩色输出图像（用于绘制结果）
#   threshold: 匹配阈值（0.0-1.0）
# 返回值：匹配结果区域或None
##############################################################################
def find_image(template, gray_img, color_img, threshold):
    # 在灰度图上执行模板匹配
    result = gray_img.find_template(
        template, threshold, step=4, search=SEARCH_EX
    )

    if result:
        # 在彩色图像上绘制红色矩形框标记匹配位置
        color_img.draw_rectangle(result, color=(255, 0, 0))
    return result

##############################################################################
# 函数名：find_pattern
# 功能：识别道路模式（左转、右转、分支）
# 参数：
#   gray_img: 灰度输入图像
#   color_img: 彩色输出图像
# 返回值：
#   0: 检测到左转标志
#   1: 检测到右转标志
#   2: 检测到分支标志
#   3: 未检测到任何标志
##############################################################################
def find_pattern(gray_img, color_img):
    result_code = 3  # 默认未检测到任何标志
    
    # 检测左转标志（大、中、小尺寸）
    ll_match = find_image(template_ll, gray_img, color_img, 0.6)
    if ll_match:
        print("[INFO] 检测到大型左转标志")
        result_code = 0

    lm_match = find_image(template_lm, gray_img, color_img, 0.6)
    if lm_match:
        print("[INFO] 检测到中型左转标志")
        result_code = 0

    ls_match = find_image(template_ls, gray_img, color_img, 0.6)
    if ls_match:
        print("[INFO] 检测到小型左转标志")
        result_code = 0

    # 检测右转标志（大、中、小尺寸）
    rl_match = find_image(template_rl, gray_img, color_img, 0.7)
    if rl_match:
        print("[INFO] 检测到大型右转标志")
        result_code = 1

    rm_match = find_image(template_rm, gray_img, color_img, 0.7)
    if rm_match:
        print("[INFO] 检测到中型右转标志")
        result_code = 1

    rs_match = find_image(template_rs, gray_img, color_img, 0.7)
    if rs_match:
        print("[INFO] 检测到小型右转标志")
        result_code = 1

    # 检测分支标志
    b1_match = find_image(template_b1_1, gray_img, color_img, 0.8)
    if b1_match:
        print("[INFO] 检测到分支标志 1")
        result_code = 2

    b2_match = find_image(template_b2_1, gray_img, color_img, 0.8)
    if b2_match:
        print("[INFO] 检测到分支标志 2")
        result_code = 2
        
    return result_code

##############################################################################
# 函数名：find_obstacle
# 功能：检测障碍物
# 参数：
#   gray_img: 灰度输入图像
#   color_img: 彩色输出图像
# 返回值：布尔值，True表示检测到障碍物，False表示未检测到
##############################################################################
def find_obstacle(gray_img, color_img):
    # 检测三种障碍物模板
    o1_match = find_image(template_o1, gray_img, color_img, 0.45)
    if o1_match:
        print("[WARNING] 检测到障碍物 1")

    o2_match = find_image(template_o2, gray_img, color_img, 0.45)
    if o2_match:
        print("[WARNING] 检测到障碍物 2")

    o3_match = find_image(template_o3, gray_img, color_img, 0.45)
    if o3_match:
        print("[WARNING] 检测到障碍物 3")

    # 返回是否检测到任何障碍物
    return o1_match or o2_match or o3_match