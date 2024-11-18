#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Import các thư viện cần thiết
import rospy
from geometry_msgs.msg import Twist           # Thư viện ROS để nhận lệnh vận tốc
from std_msgs.msg import Int16MultiArray      # Để gửi dữ liệu dạng mảng số nguyên
import time

import RPi.GPIO as GPIO                       # Thư viện GPIO để điều khiển chân I/O của Raspberry Pi
import serial                                 # Thư viện để giao tiếp qua cổng serial
import sys
from math import pi                           # Sử dụng số pi cho tính toán góc và vận tốc

class ControlMotorByKinematic():
    def __init__(self):
        # Khởi tạo node ROS tên 'control_motor' 
        rospy.init_node('control_motor', anonymous=False)
        self.rate = rospy.Rate(30)  # Tần số thực hiện vòng lặp là 30 Hz

        # Các tham số cấu hình cổng serial và tốc độ truyền dữ liệu
        self.SERIAL_PORT = rospy.get_param("SERIAL_PORT", '/dev/ttyS0')
        self.BAUD_RATE = rospy.get_param("BAUD_RATE", 19200)

        # ID của các động cơ trái và phải
        self.id_motorLeft = rospy.get_param("id_motorLeft", 1)
        self.id_motorRight = rospy.get_param("id_motorRight", 2)

        # Hướng mặc định của động cơ trái và phải
        self.dir_motorLeft = 0
        self.dir_motorRight = 1

        # Các tham số vật lý của hệ thống
        self.r_banh = rospy.get_param("r_banh", 0.0725)           # Bán kính bánh xe
        self.kc_hai_banh = rospy.get_param("kc_hai_banh", 0.44)   # Khoảng cách giữa hai bánh xe
        self.v_max_dong_co = rospy.get_param("v_max_dong_co", 14000) # Tốc độ tối đa của động cơ
        self.pwm_max = rospy.get_param("pwm_max", 255)            # PWM tối đa
        self.hop_giam_toc = rospy.get_param("hop_giam_toc", 71.2) # Tỉ số giảm tốc của hộp số

        # Đăng ký callback để nhận lệnh vận tốc từ topic /cmd_vel
        rospy.Subscriber("/cmd_vel", Twist, self.cmdVel_callback)	
        self.data_cmdVel = Twist()    # Biến lưu trữ dữ liệu vận tốc mới nhận
        self.is_cmdVel = 0            # Cờ kiểm tra có dữ liệu vận tốc mới

        # Mở kết nối serial với Raspberry Pi
        try:
            self.ser = serial.Serial(self.SERIAL_PORT, self.BAUD_RATE)
            time.sleep(1)  # Chờ một chút để kết nối ổn định
            print("Open serial successfully!")  # Thông báo mở serial thành công
        except Exception as e:
            print("Error when create Serial, Error: ", e)  # Thông báo lỗi mở serial
            sys.exit()

    # Hàm callback khi có dữ liệu mới từ topic /cmd_vel
    def cmdVel_callback(self, data):
        self.data_cmdVel = data
        self.is_cmdVel = 1

    # Hàm chuyển vận tốc góc bánh xe thành giá trị PWM
    def convert_pwm(self, v_banh):
        pwm = v_banh * self.hop_giam_toc
        pwm = (pwm / self.v_max_dong_co) * self.pwm_max
        return int(pwm)

    # Hàm tính toán vận tốc góc của các bánh từ vận tốc và góc quay của xe
    def donghocnghich(self, v_dai, v_goc):
        vtgoc_trai = (1.0 / (2.0 * self.r_banh)) * (2.0 * v_dai - self.kc_hai_banh * v_goc)
        vtgoc_phai = (1.0 / (2.0 * self.r_banh)) * (2.0 * v_dai + self.kc_hai_banh * v_goc)

        print(v_dai, v_goc)

        v_banh_trai = self.convert_pwm(vtgoc_trai * (60 / (2 * pi)))  # Chuyển đổi thành vòng/phút
        v_banh_phai = self.convert_pwm(vtgoc_phai * (60 / (2 * pi)))

        return v_banh_trai, v_banh_phai

    # Hàm gửi lệnh PWM qua serial
    def sendPWM(self, _id = 0, _pwm = 0, _dir = 0):        
        frame_send = bytes([_dir<<7|_id, _pwm, 0xFF])
        print(f"Gửi: {frame_send.hex()}")  # In ra lệnh đã gửi
        self.ser.write(frame_send)
    
    # Hàm điều khiển chính
    def run(self):
        while not rospy.is_shutdown():
            if self.is_cmdVel == 1:
                # Tính toán giá trị PWM cho bánh trái và bánh phải
                pwm_v_trai, pwm_v_phai = self.donghocnghich(self.data_cmdVel.linear.x, self.data_cmdVel.angular.z)
                if pwm_v_trai == 0 and pwm_v_phai == 0:
                    pwm_v_trai = 3
                    pwm_v_phai = 3

                print(pwm_v_trai, pwm_v_phai)

                try:
                    # Thiết lập hướng và PWM cho bánh trái
                    _dir_left = self.dir_motorLeft
                    if pwm_v_trai < 0:
                        _dir_left = 1 - _dir_left

                    _pwm_left = abs(pwm_v_trai)

                    # Thiết lập hướng và PWM cho bánh phải
                    _dir_right = self.dir_motorRight
                    if pwm_v_phai < 0:
                        _dir_right = 1 - _dir_right

                    _pwm_right = abs(pwm_v_phai)

                    # Gửi lệnh PWM qua serial
                    self.sendPWM(self.id_motorLeft, _pwm_left, _dir_left)
                    time.sleep(0.01)
                    self.sendPWM(self.id_motorRight, _pwm_right, _dir_right)
                    time.sleep(0.01)

                except:
                    pass

            self.rate.sleep()

        # Đóng kết nối serial
        self.ser.close()

# Hàm main để khởi động chương trình
def main():
    print('Starting main program')
    controlMT = ControlMotorByKinematic()
    controlMT.run()

# Chạy chương trình nếu script được gọi trực tiếp
if __name__ == '__main__':
    main()

