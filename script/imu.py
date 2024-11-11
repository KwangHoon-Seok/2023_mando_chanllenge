#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import Imu
import tf
import os
from tf.transformations import euler_from_quaternion
from math import pi
from std_msgs.msg import Float32
import PID
class IMUParser:
    def __init__(self):
        rospy.init_node('imu', anonymous=True)
        self.image_sub = rospy.Subscriber("/imu", Imu, self.callback)
        self.is_imu = False
        self.yaw_pub = rospy.Publisher("imu_yaw",Float32,queue_size=1)
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            os.system('clear')
            if not self.is_imu:
                print("[1] can't subscribe '/imu' topic... \n    please check your IMU sensor connection")

            self.is_imu = False
            rate.sleep()

    def callback(self,data):
        self.is_imu = True
        quaternion=(data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w)
        roll,pitch,yaw = euler_from_quaternion(quaternion)
        
        roll_deg=roll / pi * 180
        pitch_deg=pitch / pi * 180
        yaw_deg=yaw / pi * 180

        os.system('clear')
        print(f'''
        --------------[ IMU data ]---------------
             Roll  (deg) = {roll_deg}
             Pitch (deg) = {pitch_deg}
             Yaw   (deg) = {yaw_deg}
        -----------------------------------------
        ''')
        self.prev_time = rospy.get_rostime()
        self.yaw_pub.publish(yaw_deg)


if __name__ == '__main__' :
    try:
        imu_parser = IMUParser()
    except rospy.ROSInterruptException:
        pass