#!/usr/bin/env python3
# -*- coding: utf-8 -*- 16
import rospy
from morai_msgs.msg import GPSMessage
from std_msgs.msg import Float32
import pandas as pd
import math
import os
import rospkg

from pyproj import Proj
class imu_yaw():
    def __init__(self):
        rospy.init_node('get_yaw')
        
        rospy.Subscriber('/gps', GPSMessage, self.cb_gps, queue_size=1)
        #rospy.Subscriber('/imu_yaw', Float32, self.cb_imu)
        self.yaw_data = []  # Yaw 데이터를 저장할 리스트
        self.ref_list = []
        self.proj_UTM = Proj(proj='utm', zone=52, ellps='WGS84', preserve_units=False)
        self.x,self.y=0,0

        # ROS 노드가 종료될 때 실행될 함수를 등록합니다.
        rospy.on_shutdown(self.save_yaw_data_to_csv)
        
    def main(self):
        loop_Hz = 50
        rate = rospy.Rate(loop_Hz)
        px, py = self.x, self.y
        rospy.loginfo('dky_driver: waiting for gps topics')
        #rospy.wait_for_message('/imu_yaw', Float32)
        rospy.loginfo('dku_driver: begin')
        
        while not rospy.is_shutdown():
            if math.sqrt((px-self.x)**2+(py-self.y)**2) > 0.1:
                if hasattr(self, 'yaw'):
                    print(self.yaw)
                    self.ref_list.append([self.x, self.y])
                    #self.yaw_data.append(self.yaw)
                    px, py = self.x, self.y
                    rospy.loginfo(f'({self.x:0.1f}, {self.y:0.1f})')
                    #print(len(self.ref_list), len(self.yaw_data))
            rate.sleep()
    
    def cb_imu(self, data):
        self.yaw = data.data  # data 필드에서 실제 yaw 값을 추출
    
    def cb_gps(self,data):
        self.x, self.y = self.proj_UTM(data.longitude, data.latitude)
    
    # def save_yaw_data_to_csv(self):
    #     # pandas DataFrame을 사용하여 GPS 좌표와 Yaw 데이터를 저장하고, csv 파일로 출력
    #     df = pd.DataFrame(self.ref_list, columns=['X', 'Y', 'Yaw'])
    #     df.to_csv('/home/owner/catkin_ws/src/2023_dku/ref_path/imu.csv', index=False)
    #     rospy.loginfo('GPS and Yaw data saved to imu_gps_yaw.csv')
        
    def save_yaw_data_to_csv(self):
         
        for i in range(len(self.yaw_data)):
            self.ref_list[i].append(self.yaw_data[i])
        # X, Y, Yaw 세 열을 가진 DataFrame을 생성합니다.
        df = pd.DataFrame(self.ref_list)

        # 생성된 DataFrame을 CSV 파일로 저장합니다.
        try:
            df.to_csv('/home/owner/catkin_ws/src/2023_dku/script/gps11.csv', index=False)
            rospy.loginfo('GPS and Yaw data saved to imu.csv')
        except Exception as e:
            rospy.loginfo(f'Error saving to CSV: {e}')

if __name__ == '__main__':
    morai_imu = imu_yaw()
    morai_imu.main()
