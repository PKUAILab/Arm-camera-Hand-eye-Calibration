import time
import numpy as np
from HitbotInterface import HitbotInterface

print("-------------init the arm----------------")
robot_id = 18
robot = HitbotInterface(robot_id)
robot.net_port_initial()
time.sleep(2)
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
    
    time.sleep(1)
    robot.set_drag_teach(True)

    while(1):
        time.sleep(0.1)
        robot.get_scara_param() 
        tarm = np.array([robot.x, robot.y, robot.z,robot.r]).T
        print(tarm)