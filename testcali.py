import time
import numpy as np
from HitbotInterface import HitbotInterface


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

    r_base2cam = [[ 9.99997455e-01,-2.03566270e-03,9.72935607e-04],
    [ 2.04645977e-03 ,9.99934866e-01,-1.12283445e-02],
    [-9.50015113e-04 ,1.12303070e-02,9.99936487e-01]]
    t_base2cam = [[0.00000000e+00],
    [2.83212455e+20],
    [8.20724535e+19]]
    

    campoint1 = np.array([167,236,0],dtype='float32')
    armpoint1 = np.dot(campoint1, np.array(r_base2cam)) + np.array(t_base2cam,dtype='float32').T
    print(armpoint1)
    armpoint1 = armpoint1[0]
    robot.new_movej_xyz_lr(armpoint1[0], armpoint1[1], armpoint1[2], 0, 80, 0, -1)