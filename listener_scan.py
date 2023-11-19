#!/usr/bin/env python3

import rospy
# from std_msgs.msg import String
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import String, Float32, Int64
import math
import numpy as np
from geometry_msgs.msg import Twist

class Turtle():
    def __init__(self):
        self.fov = 180

        self.st_angle = 10
        self.distance = 0.20

        self.dt = 0
        self.new_time, self.start_time = 0, rospy.get_time()
        self.s, self.v, self.a = np.zeros((3)), np.zeros((3)), np.zeros((3))
        self.key = ""
        self.cmd_angle = ""
        self.cmd_forward = ""

    def keyboard_callback(self, data):
        self.key = data.data
        

    def scan_callback(self, msg):
        header = msg.header
        angle_min = msg.angle_min  # Minimum angle (in radians) of the scan
        angle_max = msg.angle_max  # Maximum angle (in radians) of the scan
        angle_increment = msg.angle_increment  # Angular increment between measurements
        range_min = msg.range_min  # Minimum range value (in meters)
        range_max = msg.range_max  # Maximum range value (in meters)
        ranges = np.array(msg.ranges)  # List of range measurements at various angles (360 degree)


        ranges[self.fov//2 : 360 - self.fov//2] = 99
        ranges[ranges == 0] = 99

        min_angle = np.argmin(ranges)
        min_d = ranges[np.argmin(ranges)]


        print(f"Min {min_angle} D: {min_d}")

        if min_angle < self.st_angle or min_angle > 360 - self.st_angle:
            self.cmd_angle = "0"

            if min_d > self.distance:
                self.cmd_forward = "+"
            else:
                self.cmd_forward = "0"

        elif min_angle > 0 and min_angle < 180:
            self.cmd_angle = "+"
        elif min_angle >= 180:
            self.cmd_angle = "-"

        print(self.cmd_angle)


def main():
    rospy.init_node('listener_scan', anonymous=True)
    np.set_printoptions(suppress=True)

    myturtle = Turtle()
    rospy.Subscriber('keyboard_input', String, myturtle.keyboard_callback)
    rospy.Subscriber('/scan', LaserScan, myturtle.scan_callback)

    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(10) # 10hz

    cmd = Twist()
    while not rospy.is_shutdown():

        # if myturtle.key == 'a':
        #     cmd.angular.z = 0.5
        # if myturtle.key == 'd':
        #     cmd.angular.z = -0.5
        # if myturtle.key == 's':
        #     cmd.angular.z = 0

        if myturtle.cmd_angle == '+':
            cmd.angular.z = 1.1
        elif myturtle.cmd_angle == '-':
            cmd.angular.z = -1.1
        elif myturtle.cmd_angle == '0':
            cmd.angular.z = 0


        if myturtle.cmd_forward == '+':
            cmd.linear.x = 0.20
        elif myturtle.cmd_angle == '0':
            cmd.linear.x = 0


        pub.publish(cmd)
        rate.sleep()

    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

