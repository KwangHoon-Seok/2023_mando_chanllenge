
#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
from stanley import EgoState, Stanley
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import numpy as np
from nav_msgs.msg import Path
from std_msgs.msg import Float32
from carla_msgs.msg import CarlaEgoVehicleControl
from carla_msgs.msg import CarlaSpeedometer
import math
import time
from pid import PID

class Drive:
    def __init__(self):
        self.x_vals = []
        self.y_vals = []
        
        self.heading = 0
        self.x = 0 
        self.y = 0
        self.cur_speed = 0
        #---------------------------acc variables----------------------------------#
        self.flag_acc = 0
        self.target_distance = 0
        self.front_obstacle = 0
        self.time_headway = 3
        self.pid_acc = PID(1,0.03,0.01)
        #--------------------------------------------------------------------------#
        self.stanley_controller = Stanley()
        self.ego = EgoState()
        self.pid_v = PID(1,0.03,0.01)  # PID 인스턴스 생성
        rospy.init_node('tracking_node', anonymous=True)
        rospy.Subscriber('/carla/ego_vehicle/odometry', Odometry, self.odometry_callback)
        rospy.Subscriber('/carla/ego_vehicle/speedometer', CarlaSpeedometer, self.speed_callback)
        self.pub = rospy.Publisher('/carla/ego_vehicle/vehicle_control_cmd_manual', CarlaEgoVehicleControl, queue_size=5)
        self.control_msg = CarlaEgoVehicleControl()
    
    #-------------------------callback functions start-------------------------------#
    def odometry_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        quaternion = (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        )
        euler_angles = euler_from_quaternion(quaternion)
        self.heading = euler_angles[2]
        self.heading = self.heading * 180 / np.pi 

    def speed_callback(self, msg):
        self.cur_speed = msg.speed * 3.6  # m/s를 km/h로 변환

    #-------------------------callback functions finish-------------------------------#
    
    def run(self):
        self.stanley_controller.set_ref_path('odometry_data.csv')  # 경로 설정
        target_speed = int(input("목표 속도 설정 (km/h): "))
        rate = rospy.Rate(20)  # 20Hz로 루프 실행
        
        while not rospy.is_shutdown():
            self.ego.x = self.x
            self.ego.y = self.y
            self.ego.heading = self.heading
            self.ego.velocity = self.cur_speed  # 현재 속도를 업데이트

            self.stanley_controller.set_state(self.ego)
            steer = self.stanley_controller.main(self.cur_speed)

            speed_error = target_speed - self.cur_speed
            target_distance = self.time_headway * self.cur_speed
            distance_error = target_distance - self.front_obstacle

            if self.flag_acc == 0:
                throttle = self.pid_v.compute(speed_error)
                throttle = max(0, min(throttle, 1))  # throttle 범위 제한 (0~1)

                if speed_error < 0:
                    self.control_msg.throttle = 0
                    self.control_msg.brake = max(0, min(-speed_error / target_speed, 1))  # 브레이크 범위 제한 (0~1)
                else:
                    self.control_msg.brake = 0
                    self.control_msg.throttle = throttle
            
                self.control_msg.steer = steer
                
                self.pub.publish(self.control_msg)

                # rate.sleep()
                time.sleep(0.01)  # 10ms 대기
            elif self.flag_acc == 1:
                throttle = self.pid_acc.compute(distance_error)
                throttle = max(0, min(throttle,1))

                if distance_error < 0:
                    self.control_msg.throttle = 0
                    self.control_msg.brake = max(0, min(-distance_error / target_distance, 1))
                else:
                    self.control_msg.brake = 0
                    self.control_msg.throttle = throttle
                self.control_msg.steer = steer
                self.pub.publish(self.control_msg)
                time.sleep(0.01)

if __name__ == '__main__':
    drive = Drive()
    drive.run()

