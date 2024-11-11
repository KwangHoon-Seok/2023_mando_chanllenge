#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import Float32

class ConstantTimeGapController:
    def __init__(self, time_gap, initial_velocity):
        # 시간 간격(초)
        self.time_gap = time_gap
        # 차량의 초기 속도(m/s)
        self.current_velocity = initial_velocity
        # 앞 차량과의 현재 거리(m)
        self.current_distance = 0
        # 앞 차량의 속도(m/s)
        self.lead_vehicle_speed = 0
        
       
        rospy.init_node('constant_time_gap_controller')
        
        rospy.Subscriber('/lead_vehicle_distance', Float32, self.update_distance)
        rospy.Subscriber('/lead_vehicle_speed', Float32, self.update_lead_vehicle_speed)
        
       
        self.velocity_publisher = rospy.Publisher('/ego_vehicle_velocity', Float32, queue_size=1)
        
    def update_distance(self, data):
        self.current_distance = data.data
        
    def update_lead_vehicle_speed(self, data):
        self.lead_vehicle_speed = data.data
        
    def compute_velocity(self):
        # 목표 거리 계산
        desired_distance = self.lead_vehicle_speed * self.time_gap
        
        # 속도 조정 필요 여부 판단(PI 제어로 해볼 예정)
        if self.current_distance > desired_distance:
            # 앞 차량과의 거리가 목표 거리보다 큰 경우, 속도를 높임
            self.current_velocity += 1  
        elif self.current_distance < desired_distance:
            # 앞 차량과의 거리가 목표 거리보다 작은 경우, 속도를 낮춤
            self.current_velocity -= 1  
        
       
        self.velocity_publisher.publish(self.current_velocity)
        
    def run(self):
        rate = rospy.Rate(40)  
        while not rospy.is_shutdown():
            self.compute_velocity()
            rate.sleep()

if __name__ == '__main__':
    controller = ConstantTimeGapController(time_gap=2.0, initial_velocity=30.0)
    try:
        controller.run()
    except rospy.ROSInterruptException:
        pass
