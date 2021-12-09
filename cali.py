import cv2
import math
import time

import numpy as np

import pyrealsense2 as rs
from scipy.io import savemat
from HitbotInterface import HitbotInterface

def eulerAngles2rotationMat(theta1, format='degree'):           # 角度 to 旋转矩阵
    """
    Calculates Rotation Matrix given euler angles.
    :param theta: 1-by-3 list [rx, ry, rz] angle in degree
    :return:
    RPY角，是ZYX欧拉角，依次 绕定轴XYZ转动[rx, ry, rz]
    """
    theta = [theta1, 0, 0]
    if format is 'degree':
        theta = [i * math.pi / 180.0 for i in theta]
 
    R_x = np.array([[1, 0, 0],
                    [0, math.cos(theta[0]), -math.sin(theta[0])],
                    [0, math.sin(theta[0]), math.cos(theta[0])]
                    ])
 
    R_y = np.array([[math.cos(theta[1]), 0, math.sin(theta[1])],
                    [0, 1, 0],
                    [-math.sin(theta[1]), 0, math.cos(theta[1])]
                    ])
 
    R_z = np.array([[math.cos(theta[2]), -math.sin(theta[2]), 0],
                    [math.sin(theta[2]), math.cos(theta[2]), 0],
                    [0, 0, 1]
                    ])
    R = np.dot(R_z, np.dot(R_y, R_x))
    return R

def findcorners(w1, h1, img):                   # 寻找图像角点corners
    
    ret, corners = cv2.findChessboardCorners(img, (w1,h1), None)
    #cv2.cornerSubPix(img, corners, (11, 11), (-1, -1), None)
    cv2.drawChessboardCorners(img, (w1,h1), corners, ret)
    cv2.imwrite("imgwithcorners.png", img)
    return corners

def init3dpoints(w1, h1, radius=30):    # 生成棋盘格世界坐标，radius为棋盘格距离   shape[w1*h1, 3]
    objpoints = np.zeros((w1*h1, 3))
    for j in range(h1):
        for i in range(w1):
            objpoints[w1 * j + i] = [j*radius, i*radius, 0]
    return np.array(objpoints, dtype='float32')

def getcammatrix(img, w1, h1, cameraMatrix, distCoeffs):
    objpoints = init3dpoints(w1, h1, radius=30)
    # obj_corner = np.zeros([h1 * w1, 3], np.float32)
    # obj_corner[:, :2] = np.mgrid[0:h1, 0:w1].T.reshape(-1, 2)  # (w*h)*2
    imgpoints = findcorners(h1, w1, img)
    imgpoints = np.array(imgpoints, dtype='float32')
    print(imgpoints.shape)
    
    assert objpoints.shape[0] == imgpoints.shape[0]
    _, rvec, tvec = cv2.solvePnP(objpoints, imgpoints, cameraMatrix, distCoeffs, flags=1)
    # print("rvec = {}".format(rvec))
    # print("tvec = {}".format(tvec))
    R = cv2.Rodrigues(rvec)             # 第一个是旋转矩阵，第二个是 雅可比矩阵
    T = tvec
    return R[0], T


if __name__ == '__main__':
    print("-------------init the arm----------------")
    robot_id = 18
    robot = HitbotInterface(robot_id)
    robot.net_port_initial()
    time.sleep(3)
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
        robot.new_movej_xyz_lr(90, 90, -110, 320, speed=30, roughly=7, lr=1)
        time.sleep(1)
        

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


    print("---------------init params---------------")
    w1 = 8
    h1 = 11
    i = 0                                           # i表示第几个图像
    movetime = 10                                    # 留给人的移动机械臂的时间 s
    cameraMatrix = np.array([[9.087423095703125000e+02,0.000000000000000000e+00,6.349531860351562500e+02],
                            [0.000000000000000000e+00,9.086240234375000000e+02,3.688071594238281250e+02],
                            [0.000000000000000000e+00,0.000000000000000000e+00,1.000000000000000000e+00]])
    
    distCoeffs = np.array([0, 0, 0, 0])
    r_end2base = []
    t_end2base = []
    r_board2cam = []
    t_board2cam = []
    movepoint = [[60, -198, 20, 0],
                 [180, -180, 20, 200],
                 [230, -160, 20, 74],
                 [240, -35, 20, 22],
                 [180, -48, 20, 245],
                 [105, -60, 20, 144],
                 [233, 80, 20, 176]]                                                                                                                                                                                                                                         
    
    print("R=", robot.r)
    for i in range(3):
        
        print("-----------start circle {}------------".format(i))
        
        print("you have {} seconds to move the arm".format(movetime))
        robot.new_movej_xyz_lr(60, -198, 20, 0, speed=30, roughly=7, lr=1)
        time.sleep(movetime / 2)
        print("you have {} seconds to move the arm".format(movetime/2))
        time.sleep(movetime / 2)
        
        print("-----getting img-----")
        frames = pipeline.wait_for_frames()     # 得到图片
        color_frame = frames.get_color_frame()
        color_image = np.asanyarray(color_frame.get_data())
        cv2.imwrite("1.jpg", color_image)

        print("-----getting matrix-----")
        rcam, tcam = getcammatrix(color_image, w1, h1, cameraMatrix, distCoeffs)        # 相机矩阵
        robot.get_scara_param()                                                         # 机械臂矩阵
        theta = robot.r
        rarm = eulerAngles2rotationMat(theta)
        tarm = np.array([robot.x, robot.y, robot.z]).T
        print("tarm is {}".format(tarm))
        print("-----saving-----")
        r_end2base.append(rarm)
        t_end2base.append(tarm)
        r_board2cam.append(rcam)
        t_board2cam.append(tcam)
        time.sleep(0.5)

    print("-----calculating calibration matrix")
    print("r_end2base",r_end2base[2].shape)
    print("r_board2cam",r_board2cam[2].shape)
    print("t_end2base",t_end2base[2].shape)
    print("t_board2cam",t_board2cam[2].shape)
    r_base2cam, t_base2cam = cv2.calibrateHandEye(r_end2base, t_end2base, r_board2cam, t_board2cam)
    time.sleep(0.5)
    print("r_base2cam is {}".format(r_base2cam))
    print("t_base2cam is {}".format(t_base2cam))

    