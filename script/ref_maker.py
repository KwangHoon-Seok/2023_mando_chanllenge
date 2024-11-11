#!/usr/bin/env python3
# -*- coding: utf-8 -*- 16
from morai_msgs.msg import GPSMessage

import rospy
import math
import os
import rospkg
from pyproj import Proj
import csv
from std_msgs.msg import Float32
class RefMaker():
    def __init__(self):
        rospy.init_node('ref_maker')
        #----------------- subscribers -----------------#
        rospy.Subscriber('/gps', GPSMessage, self.cb_gps, queue_size=1)
        #rospy.Subscriber('/imu_yaw', Float32, self.cb_imu)
        self.proj_UTM = Proj(proj='utm', zone=52, ellps='WGS84', preserve_units=False)
        self.x,self.y=0,0

        rospy.on_shutdown(self.save)
    def main(self):
        loop_hz = 50
        rate = rospy.Rate(loop_hz)
        rospy.wait_for_message('/gps', GPSMessage)
        px, py = self.x, self.y
        self.ref_list = []
        self.yaw_data = [] 
        while not rospy.is_shutdown():
            if math.sqrt((px-self.x)**2+(py-self.y)**2) > 0.1:
                self.ref_list.append([self.x, self.y])
                px, py = self.x, self.y
                rospy.loginfo(f'({self.x:0.1f}, {self.y:0.1f})')
            rate.sleep()

    def save(self):
        #fn = 'ref_course_3'
        rospack = rospkg.RosPack()
        filename = os.path.join(rospack.get_path('dku_morai_driver'), 
                                                 f'script/test.csv')
        with open(filename, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            for row in self.ref_list:
                writer.writerow(row)

    def cb_gps(self, data):
        x, y = self.proj_UTM(data.longitude, data.latitude)
        self.x = x - data.eastOffset
        self.y = y - data.northOffset
if __name__ == '__main__':
    rm = RefMaker()
    rm.main()