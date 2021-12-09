# -*- coding:utf-8 -*-
import os
import cv2
import sys
import time
import random
import argparse
import numpy as np
from serialcontrol import pump_off
import torch

sys.path.append(os.path.dirname(__file__))
from HitbotInterface import HitbotInterface
# from form2fit.code.get_align_img import initial_camera,get_curr_image
import pyrealsense2 as rs        

def rand_coords(radius=6400):   
    randcoords = []                                                         # 存放生成的坐标
    # sqs = []                                                                # 暂存x y 的点值
    x1 = random.uniform(95, 250)
    y1 = random.uniform(-200, -40)
    sq1 = (x1 ** 2) + (y1 ** 2)
    randomrz = int(random.uniform(0, 90))                                   # 随机生成坐标,以及旋转角度，输出[x,y,z], rz
    randcoords.append(((int(x1), int(y1), -92), randomrz))
    while True:
        x2 = random.uniform(95, 250)
        y2 = random.uniform(-200, -40)
        sq2 = (x2 ** 2) + (y2 ** 2)
        if ((abs(x2 - x1)) ** 2 + (abs(y2 - y1)) ** 2) >= radius:
            randomrz = int(random.uniform(0, 90))
            randcoords.append(((int(x2), int(y2), -92), randomrz))
            break
    while True:
        x3 = random.uniform(95, 250)
        y3 = random.uniform(-200, -40)
        sq3 = (x3 ** 2) + (y3 ** 2)
        if ((abs(x3 - x1)) ** 2 + (abs(y3 - y1)) ** 2) >= radius and ((abs(x3 - x2)) ** 2 + (abs(y3 - y2)) ** 2) >= radius:
            randomrz = int(random.uniform(0, 90))
            randcoords.append(((int(x3), int(y3), -92), randomrz))
            break
    while True:
        x4 = random.uniform(95, 250)
        y4 = random.uniform(-200, -40)
        sq4 = (x4 ** 2) + (y4 ** 2)
        if ((abs(x4 - x1)) ** 2 + (abs(y4 - y1)) ** 2) >= radius and ((abs(x4 - x2)) ** 2 + (abs(y4 - y2)) ** 2) >= radius and ((abs(x4 - x3)) ** 2 + (abs(y4 - y3)) ** 2) >= radius:
            randomrz = int(random.uniform(0, 90))
            randcoords.append(((int(x4), int(y4), -92), randomrz))
            break
                                                   
       
    return randcoords

def find_coords():                                                           #随机生成选取坐标，(四选一)
    x = [0,1,2,3]
    random.shuffle(x)                                                      
    return x


def world2pixel(worldpiont):                                                #待补充
    time.sleep(1)
    pixel = 0
    return pixel

def judge():                                                #是否被成功吸取（是否重量发生相应变化）
    time.sleep(1)                                                           #待补充
    judgenum = 1
    return judgenum

def coordup(coord):                                         #计算吸取点上方坐标
    coord = coord
    coord[2] = coord[2] + 85
    return coord

def coorddown(coord):                                       #计算吸取点上方坐标
    coord = coord
    coord[2] = coord[2] - 85
    return coord

def auto_collection():
             
    frames = pipeline.wait_for_frames()     # 得到图片
    color_frame = frames.get_color_frame()
    color_image = np.asanyarray(color_frame.get_data())
    cv2.imwrite("assets/color{}.png".format(batch*2 + 0), color_image)
    cv2.imwrite("assets/depth{}.png".format(batch*2 + 0), color_image)

    chosenum = findcoords[batch]
    coord1, name = coordlist[chosenum]                  # 四选一 [x, y, z]三维坐标 coord1
    print("ready to suck {}".format(name))
    time.sleep(0.5)
    points = []                                         # points收集所有放置（数据集中的吸取）位置   --机械臂坐标系
    camcoord1 = world2pixel(coord1)
    points.append(camcoord1)                            # coord1 三维坐标对应的像素坐标 camcoord1

    a = robot.new_movej_xyz_lr(coord1[0], coord1[1], coord1[2] + 40, 150,80,0,1)      # 机械臂准备吸取
    robot.wait_stop()
    if a == 1: print("moving") 
    else: print("error, code is {}".format(a))
    time.sleep(0.5)  
                                   
    a = robot.new_movej_xyz_lr(coord1[0], coord1[1], coord1[2], 150,90, 0,  1)         # 机械臂吸取
    robot.wait_stop()
    if a == 1: print("moving") 
    else: print("error, code is {}".format(a)) 
    time.sleep(0.5)  
    # coord1up = coordup(coord1)
    a = robot.new_movej_xyz_lr(coord1[0], coord1[1], coord1[2] + 40, 150,90,0,1)       # 机械臂抬起
    robot.wait_stop()
    if a == 1: print("moving") 
    else: print("error, code is {}".format(a))
    judgenum = judge()
    if judgenum == 0:
        print("The suction failed! Abord")
        
        # raise RuntimeError 
    else:
        print("The suction is complete.")
        print(randcoords[batch])
        coord2, rz1= randcoords[batch]                         #随机生成坐标和角度 [x,y,z]coord2  rz1
        camcoord2 = world2pixel(coord2)
        points.append(camcoord2)

        
        time.sleep(0.5) 
        a = robot.new_movej_xyz_lr(coord2[0], coord2[1], coord2[2] + 40, rz1,80,0,-1)    # 机械臂准备放置
        robot.wait_stop()
        print("ready to place: coord value {}, speed 70\n".format(coord2))
        if a == 1: print("moving") 
        else: print("error, code is {}".format(a))
        time.sleep(0.5)
        print("::placing, coord value {}, speed 70\n".format(coord2))
        a = robot.new_movej_xyz_lr(coord2[0], coord2[1], coord2[2] + 2, rz1,60,0,-1)       # 机械臂放置
        robot.wait_stop()
        if a == 1: print("moving") 
        else: print("error, code is {}".format(a))
        time.sleep(0.5)
        pump_off()                                        # 机械臂松手
        time.sleep(0.5)

        print("::send_coords, coord value {}, speed 70\n".format(coord2))
        a = robot.new_movej_xyz_lr(coord2[0], coord2[1], coord2[2] + 40, rz1,90,0,-1)      # 机械臂抬起
        robot.wait_stop()
        if a == 1: print("moving") 
        else: print("error, code is {}".format(a))
        time.sleep(0.5)
        
        robot.new_movej_xyz_lr(150, 100, 20,    0,    70, 0,  1)                     # 机械臂回原位置
        robot.wait_stop()
        frames = pipeline.wait_for_frames()                                            
        color_frame = frames.get_color_frame()
        color_image = np.asanyarray(color_frame.get_data())                             # 记录图像
        cv2.imwrite("assets/color{}.png".format(batch*2 + 1), color_image)                 # 同为第一时刻的final和第二时刻的init图像
        cv2.imwrite("assets/depth{}.png".format(batch*2 + 1), color_image)
        angles1 = rz1                 #计算旋转角度，并存储
        angles.append(angles1)  
        time.sleep(1)

def recollection():
    coord1, rz1= randcoords[batch]                      # 得到第一个吸取位置
    chosenum = findcoords[batch]
    coord2, name = coordlist[chosenum]                  # 找到对应选择坐标的放置位置
    print("ready to replace the {}.".format(name))
    a = robot.new_movej_xyz_lr(coord1[0], coord1[1], coord1[2] + 40, rz1,80,0,-1)      # 机械臂准备吸取
    robot.wait_stop()
    if a == 1: print("moving") 
    else: print("error, code is {}".format(a))
    time.sleep(0.5)  
                                   
    a = robot.new_movej_xyz_lr(coord1[0], coord1[1], coord1[2]-7, rz1,80, 0,-1)         # 机械臂吸取
    robot.wait_stop()
    if a == 1: print("moving") 
    else: print("error, code is {}".format(a)) 
    time.sleep(1)  

    a = robot.new_movej_xyz_lr(coord1[0], coord1[1], coord1[2] + 40, rz1,90,0,-1)       # 机械臂抬起
    robot.wait_stop()
    if a == 1: print("moving") 
    else: print("error, code is {}".format(a))

    time.sleep(0.5) 
    print("ready to place: coord value {}, speed 70\n".format(coord2))
    a = robot.new_movej_xyz_lr(coord2[0], coord2[1], coord2[2] + 40, 150,80,0,1)    # 机械臂准备放置
    robot.wait_stop()
    if a == 1: print("moving") 
    else: print("error, code is {}".format(a))
    time.sleep(0.5)
    a = robot.new_movej_xyz_lr(coord2[0], coord2[1], coord2[2]+5, 150,60,0,1)       # 机械臂放置
    robot.wait_stop()
    if a == 1: print("moving") 
    else: print("error, code is {}".format(a))
    time.sleep(0.5)
    pump_off()                                            # 机械臂松手
    time.sleep(0.5)

    a = robot.new_movej_xyz_lr(coord2[0], coord2[1], coord2[2] + 40, 150,90,0,1)      # 机械臂抬起
    robot.wait_stop()
    if a == 1: print("moving") 
    else: print("error, code is {}".format(a))
    time.sleep(0.5)
    
    robot.new_movej_xyz_lr(150, 100, 20,    0,    80, 0,  1)                     # 机械臂回原位置
    robot.wait_stop()
    time.sleep(1)

if __name__ == "__main__":
    def str2bool(s):
        return s.lower() in ["1", "true"]
    parser = argparse.ArgumentParser(description="Descriptor Network Visualizer")
    parser.add_argument("--modelname", default="black-floss", type=str)
    parser.add_argument("--epochs", type=int, default=160, help="The number of training epochs.")
    opt = parser.parse_args()
    
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

    # all_data_collection

    
    a = ([95.15, 58.59, -74.39], 'cylinder')
    b = ([94.83, 135.32, -74.39], 'cube')
    c = ([172.70, 128.435, -74.39], 'triangle')
    d = ([170.81, 56.61, -74.39], 'prism')
    coordlist = (a, b, c, d)
    

    
    print("---------------init camera---------------")
    pipeline = rs.pipeline()
    config = rs.config()
    # Get device product line for setting a supporting resolution
    pipeline_wrapper = rs.pipeline_wrapper(pipeline)
    pipeline_profile = config.resolve(pipeline_wrapper)
    device = pipeline_profile.get_device()
    device_product_line = str(device.get_info(rs.camera_info.product_line))

    found_rgb = False
    for s in device.sensors:
        if s.get_info(rs.camera_info.name) == 'RGB Camera':
            found_rgb = True
            break
    if not found_rgb:
        print("The demo requires Depth camera with Color sensor")
        exit(0)

    #config.enable_stream(rs.stream.depth, 848, 480, rs.format.z16, 30)

    if device_product_line == 'L500':
        config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 10)
    else:
        config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 10)
    pipeline.start(config)                 
    
    print("-------------init the arm----------------")
    robot_id = 18
    robot = HitbotInterface(robot_id)
    robot.net_port_initial()
    time.sleep(1)
    print("initial successed")
    ret = robot.is_connect()
    while ret != 1:
        time.sleep(0.1)
        ret = robot.is_connect()
        print(ret)
    ret = robot.initial(3, 180)
    if ret == 1:
        print("robot initial successful")
        robot.unlock_position()
    else:
        print("robot initial failed")
    if robot.unlock_position():
        print("------unlock------")
    time.sleep(1)

    if robot.is_connect():
        print("robot online")
        a = robot.new_movej_xyz_lr(90, 90, -20, 0, speed=80, roughly=0, lr=1)
        print("robot statics is {}".format(a))
        time.sleep(1)



    angles = []
    for epoch in range(opt.epochs):
        print("-------------epoch{}-----------------".format(epoch))
        findcoords = find_coords()                          # 每个epoch开始时，随机生成选取坐标list，(四选一) 
        randcoords = rand_coords()                          # 随机生成放置的四个坐标list
        
        for batch in range(4):
            auto_collection()   #启动自动收集流程

        print("Placing complete. Now start to recollect the items.".format(epoch))
        time.sleep(0.5)
        for batch in range(4):
            recollection()      #启动放回流程

        print("Epoch {} complete. ".format(epoch))
        time.sleep(1.5)

