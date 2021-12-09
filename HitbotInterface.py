from ctypes import *


# do not edit this python file

class HitbotInterface:
    card_number = 0
    x = 0.0
    y = 0.0
    z = 0.0
    r = 0.0
    angle1 = 0.0
    angle2 = 0.0
    communicate_success = False
    initial_finish = False
    move_flag = False
    efg_distance = 0.0
    efg_type = 0.0
    encoder_x = 0.0
    encoder_y = 0.0
    encoder_z = 0.0
    encoder_r = 0.0
    encoder_angle1 = 0.0
    encoder_angle2 = 0.0

    def __init__(self, card_number):
        self.card_number = card_number
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.r = 0.0
        self.angle1 = 0.0
        self.angle2 = 0.0
        self.efg_dis = 0.0
        self.efg_type = 0.0

    def net_port_initial(self):
        self.dll = CDLL('small_scara_interface.dll', winmode=0)
        return self.dll.net_port_initial()

    def close_server(self):
        self.dll.robot_close_server()

    def initial(self, generation, z_trail):
        return self.dll.robot_initial(c_int(self.card_number), c_int(generation), c_float(z_trail))

    def get_scara_param(self):
        c_angle1 = c_float(0)
        c_angle2 = c_float(0)
        c_z = c_float(0)
        c_r = c_float(0)
        c_x = c_float(0)
        c_y = c_float(0)
        c_communicate_success = c_bool(False)
        c_initial_finish = c_bool(False)
        c_move_flag = c_bool(False)
        self.dll.robot_get_scara_param(c_int(self.card_number), byref(c_x), byref(c_y), byref(c_z), byref(c_angle1),
                                       byref(c_angle2), byref(c_r), byref(c_communicate_success),
                                       byref(c_initial_finish),
                                       byref(c_move_flag))

        self.x = c_x.value
        self.y = c_y.value
        self.z = c_z.value
        self.angle1 = c_angle1.value
        self.angle2 = c_angle2.value
        self.r = c_r.value
        self.communicate_success = c_communicate_success.value
        self.initial_finish = c_initial_finish.value
        self.move_flag = c_move_flag.value

    def set_arm_length(self, l1, l2):
        self.dll.robot_set_arm_length(c_int(self.card_number), c_float(l1), c_float(l2))

    def unlock_position(self):
        return self.dll.robot_unlock_position(c_int(self.card_number))

    def is_connect(self):
        return self.dll.robot_is_connect(c_int(self.card_number))

    def get_joint_state(self, joint_num):
        return self.dll.robot_get_joint_state(c_int(self.card_number), c_int(joint_num))

    def set_drag_teach(self, enable):
        return self.dll.robot_set_drag_teach(c_int(self.card_number), c_bool(enable))

    def get_drag_teach(self):
        return self.dll.robot_get_drag_teach(c_int(self.card_number))

    def set_cooperation_fun_state(self, enable):
        return self.dll.robot_set_cooperation_fun_state(c_int(self.card_number), c_bool(enable))

    def get_cooperation_fun_state(self):
        return self.dll.robot_get_cooperation_fun_state(c_int(self.card_number))

    def is_collision(self):
        return self.dll.robot_is_collision(c_int(self.card_number))

    def stop_move(self):
        return self.dll.robot_stop_move(c_int(self.card_number))

    def joint_home(self, joint_num):
        return self.dll.robot_joint_home(c_int(self.card_number), c_int(joint_num))

    def movel_xyz(self, goal_x, goal_y, goal_z, goal_r, speed):
        return self.dll.robot_movel_xyz(c_int(self.card_number), c_float(goal_x), c_float(goal_y), c_float(goal_z), c_float(goal_r), c_float(speed))

    def movej_xyz(self, goal_x, goal_y, goal_z, goal_r, speed, roughly):
        return self.dll.robot_movej_xyz(c_int(self.card_number), c_float(goal_x), c_float(goal_y), c_float(goal_z), c_float(goal_r), c_float(speed), c_float(roughly))

    def movej_angle(self, goal_angle1, goal_angle2, goal_z, goal_r, speed, roughly):
        return self.dll.robot_movej_angle(c_int(self.card_number), c_float(goal_angle1), c_float(goal_angle2), c_float(goal_z), c_float(goal_r), c_float(speed), c_float(roughly))

    def change_attitude(self, speed):
        return self.dll.robot_change_attitude(c_int(self.card_number), c_float(speed))

    def movel_xyz_by_offset(self, x_offset, y_offset, z_offset, r_offset, speed):
        return self.dll.robot_movel_xyz_by_offset(c_int(self.card_number), c_float(x_offset), c_float(y_offset), c_float(z_offset), c_float(r_offset), c_float(speed))

    def wait_stop(self):
        return self.dll.robot_wait_stop(c_int(self.card_number))

    def pause_move(self):
        return self.dll.robot_pause_move(c_int(self.card_number))

    def resume_move(self):
        return self.dll.robot_resume_move(c_int(self.card_number))

    def is_stop(self):
        return self.dll.robot_is_stop(c_int(self.card_number))

    def set_digital_out(self, io_number, io_value):
        return self.dll.robot_set_digital_io_out(c_int(self.card_number), c_int(io_number), c_int(io_value))

    def get_digital_out(self, io_number):
        return self.dll.robot_get_digital_io_out(c_int(self.card_number), c_int(io_number))

    def get_digital_in(self, io_number):
        return self.dll.robot_get_digital_io_in(c_int(self.card_number), c_int(io_number))

    def set_efg_state(self, efg_type, efg_distance):
        return self.dll.robot_set_efg_state(c_int(self.card_number), c_int(efg_type), c_float(efg_distance))

    def get_efg_state(self):
        c_efg_type = c_int(0)
        c_efg_distance = c_float(0)
        ret = self.dll.robot_get_efg_state(c_int(self.card_number), byref(c_efg_type), byref(c_efg_distance))
        self.efg_type = c_efg_type
        self.efg_distance = c_efg_distance
        return ret

    def movej_xyz_lr(self, goal_x, goal_y, goal_z, goal_r, speed, roughly, lr):
        return self.dll.robot_movej_xyz_lr(c_int(self.card_number), c_float(goal_x), c_float(goal_y), c_float(goal_z), c_float(goal_r), c_float(speed), c_float(roughly), c_int(lr))

    def new_movej_xyz_lr(self, goal_x, goal_y, goal_z, goal_r, speed, roughly, lr):
        return self.dll.robot_new_movej_xyz_lr(c_int(self.card_number), c_float(goal_x), c_float(goal_y), c_float(goal_z), c_float(goal_r), c_float(speed), c_float(roughly), c_int(lr))

    def new_set_acc(self, j1_max_acc, j2_max_acc, j3_max_acc, j4_max_acc):
        return self.dll.robot_new_set_acc(c_int(self.card_number), c_float(j1_max_acc), c_float(j2_max_acc), c_float(j3_max_acc), c_float(j4_max_acc))

    def j5_motor_zero(self):
        return self.dll.robot_j5_motor_zero(c_int(self.card_number))

    def set_j5_motor_pos(self, deg, speed):
        self.dll.robot_set_j5_motor_pos.restype = c_float
        return self.dll.robot_set_j5_motor_pos(c_int(self.card_number), c_float(deg), c_float(speed))

    def get_j5_parameter(self):
        self.dll.robot_get_j5_parameter.restype = c_float
        return self.dll.robot_get_j5_parameter(c_int(self.card_number))

    def movej_j5(self, deg, speed):
        return self.dll.robot_movej_j5(c_int(self.card_number), c_float(deg), c_float(speed))

    def get_efg_state_dji(self):
        c_efg_type = c_int(0)
        c_efg_distance = c_float(0)
        ret = self.dll.robot_get_efg_state_dji(c_int(self.card_number), byref(c_efg_type), byref(c_efg_distance))
        self.efg_type = c_efg_type
        self.efg_distance = c_efg_distance
        return ret

    def set_efg_state_dji(self, efg_type, efg_distance):
        return self.dll.robot_set_efg_state_dji(c_int(self.card_number), c_int(efg_type), c_float(efg_distance))

    def new_stop_move(self):
        return self.dll.robot_new_stop_move(c_int(self.card_number))

    def get_encoder_coor(self):
        c_x = c_float(0)
        c_y = c_float(0)
        c_z = c_float(0)
        c_angle1 = c_float(0)
        c_angle2 = c_float(0)
        c_r = c_float(0)
        self.dll.robot_get_encoder_param(c_int(self.card_number), byref(c_x), byref(c_y), byref(c_z), byref(c_angle1), byref(c_angle2), byref(c_r))
        self.encoder_x = c_x.value
        self.encoder_y = c_y.value
        self.encoder_z = c_z.value
        self.encoder_angle1 = c_angle1.value
        self.encoder_angle2 = c_angle2.value
        self.encoder_r = c_r.value

    def new_movej_angle(self, goal_angle1, goal_angle2, goal_z, goal_r, speed, roughly):
        return self.dll.robot_new_movej_angle(c_int(self.card_number), c_float(goal_angle1), c_float(goal_angle2), c_float(goal_z), c_float(goal_r), c_float(speed), c_float(roughly))


