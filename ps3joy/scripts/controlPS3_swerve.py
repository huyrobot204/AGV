#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
import time
import rospy
from std_msgs.msg import Int8
from enum import Enum
from math import degrees, pi, cos, sin, fabs

class AXES_PS3(Enum):
    trai_sang = 0
    trai_len = 1
    nut_duoi_trai = 2
    phai_sang = 3
    phai_len = 4
    nut_duoi_phai = 5

class BUTTONS_PS3(Enum):
    nhan = 0
    tron = 1
    tam_giac = 2
    vuong = 3
    trai_tren = 4
    phai_tren = 5
    trai_duoi = 6
    phai_duoi = 7
    select = 8
    start = 9
    connect = 10
    analog_trai = 11
    analog_phai = 12
    up = 13
    down = 14
    left = 15
    right = 16

class CONTROL_PS3():
    def __init__(self):
        print("ROS Initial!")
        rospy.init_node('control_ps3', anonymous=False)
        self.rate = rospy.Rate(50)

        # ------------- PARAMETER ------------- #
        self.linear_x_max  = rospy.get_param('linear_x_max', 0.6)
        self.linear_x_min   = rospy.get_param('linear_x_min', 0.05)
        self.linear_y_max  = rospy.get_param('linear_y_max', 0.6)
        self.linear_y_min   = rospy.get_param('linear_y_min', 0.05)

        self.angular_max = rospy.get_param('angular_max', 0.6)
        self.angular_min = rospy.get_param('angular_min', 0.1)
        self.acceleration = rospy.get_param('acceleration', 0.05)
        self.deceleration = rospy.get_param('deceleration', 2.)
        self.safety = rospy.get_param('safety', False)

        # -- param topic
        self.topic_joy = rospy.get_param('topic_joy', '/joy')
        self.topic_cmdvel = rospy.get_param('topic_cmdvel', '/cmd_vel')
        self.topic_enablePS3 = rospy.get_param('topic_enablePS3', '/control_ps3')

        # ------------- ROS ------------- #
        # - SUBCRIBER
        rospy.Subscriber(self.topic_joy, Joy, self.callback_joy)
        self.data_joy = Joy()
        self.is_data_joy = False

        rospy.Subscriber(self.topic_enablePS3, Int8, self.callback_enablePS3)
        self.data_enbalePS3 = 0


        # - PUBLISHER
        self.pub_cmdVel = rospy.Publisher(self.topic_cmdvel, Twist, queue_size= 40)
        self.data_pubCmdVel = Twist()

        # -- 
        self.cmd_linearXNow = 0.
        self.cmd_linearYNow = 0.
        self.cmd_angularNow = 0.
        # -
        self.curr_linear_x = 0.
        self.curr_linear_y = 0.
        self.curr_angular = 0.
        # -
        self.status_vel_linear_x = 0. # 0: khong thay doi 1: dang tang toc, 2: dang giam toc
        self.saveTimeVel_linear_x = rospy.get_time()
        self.velSt_linear_x = 0.

        self.status_vel_linear_y = 0. # 0: khong thay doi 1: dang tang toc, 2: dang giam toc
        self.saveTimeVel_linear_y = rospy.get_time()
        self.velSt_linear_y = 0.

        self.status_vel_angular = 0. # 0: khong thay doi 1: dang tang toc, 2: dang giam toc
        self.saveTimeVel_angular = rospy.get_time()
        self.velSt_angular = 0.

        self.direct_linear_x = 0
        self.direct_linear_y = 0
        self.direct_angular = 0

    # -------------------
    def callback_joy(self, data):
        self.data_joy = data
        self.is_data_joy = True
        # - up/down speed (update latter)

    def callback_enablePS3(self, data):
        self.data_enbalePS3 = data.data

    def funcDecelerationByAcc(self, time_s, v_s, v_f, a):
        denlta_time_now = rospy.get_time() - time_s
        v_re = v_s + a*denlta_time_now
        if a > 0.:
            if v_re >= v_f:
                v_re = v_f
        else:
            if v_re <= v_f:
                v_re = v_f

        return v_re
    
    def getVeloctity(self, velCmd, _statusVel, time_start, vel_saveTime):
        if _statusVel == 0:
            return velCmd
        elif _statusVel == 1:
            return self.funcDecelerationByAcc(time_start, vel_saveTime, velCmd, self.acceleration)
        elif _statusVel == 2:
            return self.funcDecelerationByAcc(time_start, vel_saveTime, velCmd, -1*self.deceleration)
        else:
            return 0.

    def controlMove(self):
        if not self.is_data_joy:
            return

        if self.safety or self.data_joy.buttons[BUTTONS_PS3.nhan.value]:
            self.status_vel_linear_x = 0
            self.status_vel_linear_y = 0
            self.status_vel_angular = 0

            self.curr_linear_x = 0.
            self.curr_linear_y = 0.
            self.curr_angular = 0.

        else:
            # - linear
                #         : 45 - 0 - (-45) | (-45) - (-90) - (-135)  | (-135) - (-180) - (135) | (135) - 90 - 45
                #trai sang: 1  - 0 - (-1)  | (-1)  - (-1)  - (-1)    |  (-1)  -    0   -   1   |   1   -  1  - 1
                #phai sang; 1  - 1 -  1    |   1   -   0   -  -1     |   -1   -    -1  -   -1  |   -1  -  0  - 1
            AXES1 = self.data_joy.axes[AXES_PS3.trai_sang.value]
            AXES2 = self.data_joy.axes[AXES_PS3.trai_len.value] 
            angle_convert = 0

            if AXES1 == 0 and AXES2 == 1:
                angle_convert = 0
            elif AXES1 == -1 and AXES2 == 0:
                angle_convert = -1*pi/2.
            elif AXES1 == 0 and AXES2 == -1:
                angle_convert = pi
            elif AXES1 == 1 and AXES2 == 0:
                angle_convert = pi/2.
            elif AXES1 == 1 and AXES2 == 1:
                angle_convert = pi/4.
            elif AXES1 == -1 and AXES2 == 1:
                angle_convert = -1*pi/4.
            elif AXES1 == -1 and AXES2 == -1:
                angle_convert = -3*pi/4.
            elif AXES1 == 1 and AXES2 == -1:
                angle_convert = 3*pi/4.
            elif AXES1 == 1 and AXES2 == -1:
                angle_convert = 3*pi/4.
            else:
                if AXES2 == 1:
                    angle_convert = AXES1*pi/4.
                    # if AXES1 < 0:
                    #     angle_convert = -1*AXES1*pi/4.
                elif AXES1 == -1:
                    angle_convert = -pi/2. + AXES2*pi/4.
                    # if AXES2 < 0:
                    #     angle_convert = -pi/2 + AXES2*pi/4.
                elif AXES2 == -1:
                    angle_convert = pi - AXES1*pi/4.
                    if AXES1 < 0:
                        angle_convert = -1*pi - AXES1*pi/4.
                elif AXES1 == 1:
                    angle_convert = pi/2 - AXES2*pi/4.
                else:
                    angle_convert = 100


            if angle_convert == 100:
                self.cmd_linearXNow = 0.
                self.cmd_linearYNow = 0.
            else:
                # print(degrees(angle_convert))
                if angle_convert < pi/2. and angle_convert > -1*pi/2.:
                    if self.direct_linear_x == 2 and self.curr_linear_x != 0.:
                        self.cmd_linearXNow = 0.
                    
                    else:
                        self.direct_linear_x = 1
                        self.cmd_linearXNow = fabs(cos(angle_convert)*self.linear_x_max)
                
                else:
                    if self.direct_linear_x == 1 and self.curr_linear_x != 0.:
                        self.cmd_linearXNow = 0.
                    
                    else:
                        self.direct_linear_x = 2
                        self.cmd_linearXNow = fabs(cos(angle_convert)*self.linear_x_max)


                if angle_convert > 0. and angle_convert < pi:
                    if self.direct_linear_y == 2 and self.curr_linear_y != 0.:
                        self.cmd_linearYNow = 0.
                    
                    else:
                        self.direct_linear_y = 1
                        self.cmd_linearYNow = fabs(sin(angle_convert)*self.linear_y_max)
                
                else:
                    if self.direct_linear_y == 1 and self.curr_linear_y != 0.:
                        self.cmd_linearYNow = 0.
                    
                    else:
                        self.direct_linear_y = 2
                        self.cmd_linearYNow = fabs(sin(angle_convert)*self.linear_y_max)

                # print(self.direct_linear_x, self.direct_linear_y)

                # self.cmd_linearXNow = cos(angle_convert)*self.linear_x_max
                # self.cmd_linearYNow = sin(angle_convert)*self.linear_y_max

            # print(angle_convert, self.cmd_linearXNow, self.cmd_linearYNow)

            # - linear

        #     # - angular
            if self.data_joy.buttons[BUTTONS_PS3.trai_tren.value] == 1 and self.data_joy.buttons[BUTTONS_PS3.phai_tren.value] == 0: # quay trai
                if self.direct_angular == 2 and self.curr_angular != 0.:
                    self.cmd_angularNow = 0.

                else:
                    self.direct_angular = 1
                    self.cmd_angularNow = self.angular_max

            elif self.data_joy.buttons[BUTTONS_PS3.phai_tren.value] == 1 and self.data_joy.buttons[BUTTONS_PS3.trai_tren.value] == 0: # quay phai
                if self.direct_angular == 1 and self.curr_angular != 0.:
                    self.cmd_angularNow = 0.

                else:
                    self.direct_angular = 2
                    self.cmd_angularNow = self.angular_max

            else:
                self.cmd_angularNow = 0.

        #     # -- linear - x
            if self.curr_linear_x < self.cmd_linearXNow and self.status_vel_linear_x != 1:
                self.status_vel_linear_x = 1
                self.velSt_linear_x = self.curr_linear_x
                self.saveTimeVel_linear_x = rospy.get_time()

            elif self.curr_linear_x > self.cmd_linearXNow and self.status_vel_linear_x != 2:
                self.status_vel_linear_x = 2
                self.velSt_linear_x = self.curr_linear_x
                self.saveTimeVel_linear_x = rospy.get_time()

            elif self.curr_linear_x == self.cmd_linearXNow:
                self.status_vel_linear_x = 0

            # -- linear - y
            if self.curr_linear_y < self.cmd_linearYNow and self.status_vel_linear_y != 1:
                self.status_vel_linear_y = 1
                self.velSt_linear_y = self.curr_linear_y
                self.saveTimeVel_linear_y = rospy.get_time()

            elif self.curr_linear_y > self.cmd_linearYNow and self.status_vel_linear_y != 2:
                self.status_vel_linear_y = 2
                self.velSt_linear_y = self.curr_linear_y
                self.saveTimeVel_linear_y = rospy.get_time()

            elif self.curr_linear_y == self.cmd_linearYNow:
                self.status_vel_linear_y = 0

        #     # -- angular
            if self.curr_angular < self.cmd_angularNow and self.status_vel_angular != 1:
                self.status_vel_angular = 1
                self.velSt_angular = self.curr_angular
                self.saveTimeVel_angular = rospy.get_time()

            elif self.curr_angular > self.cmd_angularNow and self.status_vel_angular != 2:
                self.status_vel_angular = 2
                self.velSt_angular = self.curr_angular
                self.saveTimeVel_angular = rospy.get_time()

            elif self.curr_angular == self.cmd_angularNow:
                self.status_vel_angular = 0

            self.curr_linear_x = self.getVeloctity(self.cmd_linearXNow, self.status_vel_linear_x, self.saveTimeVel_linear_x, self.velSt_linear_x)
            self.curr_linear_y = self.getVeloctity(self.cmd_linearYNow, self.status_vel_linear_y, self.saveTimeVel_linear_y, self.velSt_linear_y)
            self.curr_angular = self.getVeloctity(self.cmd_angularNow, self.status_vel_angular, self.saveTimeVel_angular, self.velSt_angular)

        sign_linear_x = 0
        if self.direct_linear_x == 1:
            sign_linear_x = 1
        elif self.direct_linear_x == 2:
            sign_linear_x = -1

        sign_linear_y = 0
        if self.direct_linear_y == 1:
            sign_linear_y = 1
        elif self.direct_linear_y == 2:
            sign_linear_y = -1

        sign_angular = 0
        if self.direct_angular == 1:
            sign_angular = 1
        elif self.direct_angular == 2:
            sign_angular = -1

        twist = Twist()
        twist.linear.x = self.curr_linear_x * sign_linear_x
        twist.linear.y = self.curr_linear_y * sign_linear_y
        twist.angular.z = self.curr_angular * sign_angular

        print("curr_linear_x: ", twist.linear.x, " |curr_linear_y: ", twist.linear.y, " |curr_angular: ", twist.angular.z)


        self.pub_cmdVel.publish(twist)

    def run(self):
        while not rospy.is_shutdown():
            # -- SEND CAN
            if self.data_enbalePS3 == 0:
                self.controlMove()

            self.rate.sleep()

def main():
    print('Program starting')
    program = CONTROL_PS3()
    program.run()
    print('Programer stopped')

if __name__ == '__main__':
    main()

  


