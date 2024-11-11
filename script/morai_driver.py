#!/usr/bin/env python3
# -*- coding: utf-8 -*- 16
from morai_msgs.msg import CtrlCmd, Lamps, GPSMessage
from morai_msgs.srv import MoraiEventCmdSrv
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32, Int16

import rospy
import numpy as np
from pyproj import Proj
from tf.transformations import euler_from_quaternion
from ref_tracker import Gain, RefTracker
import time

class EgoState():
    velocity = 0
    x = 0
    y = 0
    heading = 0

class MovingAverageFilter:
    def __init__(self, window_size):
        self.window_size = window_size
        self.values = []

    def update(self, new_value):
        self.values.append(new_value)
        if len(self.values) > self.window_size:
            self.values = self.values[1:]

    def get_moving_average(self):
        if not self.values:
            return 0
        return sum(self.values) / len(self.values)
    
class MoraiDriver():
    LAMP_NO_SIGNAL = 0
    LAMP_LEFT = 1
    LAMP_RIGHT = 2
    LAMP_EMERGENCY = 3
    GEAR_P = 1
    GEAR_R = 2
    GEAR_N = 3
    GEAR_D = 4
    MISSION_FIRST = 1
    MISSION_RAMP = 2
    MISSION_L_JA = 3
    MISSION_TRAFFIC_SIGNAL = 4
    MISSION_PARKING = 5
    MISSION_FINAL = 6
    MISSION_END = 7
    MISSION_S_JA = 8

    def __init__(self):
        rospy.init_node('dku_mando_driver')

        #----------------- subscribers -----------------#
        rospy.Subscriber('/gps', GPSMessage, self.cb_gps, queue_size=1)
        rospy.Subscriber('/imu', Imu, self.cb_imu, queue_size=1)
        rospy.Subscriber('/dist_ahead', Float32, self.cb_dist_ahead, queue_size=1)
        #rospy.Subscriber('/yolo_cls', Int16, self.cb_yolo_cls, queue_size=1)
        rospy.Subscriber('/velocity', Float32, self.cb_velocity, queue_size=1)
        #rospy.Subscriber('/lane_center', Float32, self.cb_lane, queue_size=1)
        #----------------- publishers ------------------#
        self.pub_ctrl_cmd = rospy.Publisher('ctrl_cmd', CtrlCmd, queue_size=1)
        #----------------- services --------------------#
        self.srv_event_cmd = rospy.ServiceProxy('Service_MoraiEventCmd', MoraiEventCmdSrv)

        self.event_cmd_srv_type = MoraiEventCmdSrv()
        self.event_cmd_srv_type.ctrl_mode = 0
        self.event_cmd_srv_type.gear = -1
        self.event_cmd_srv_type.set_pause = False
        self.event_cmd_srv_type.lamps = Lamps()
        self.event_cmd_srv_type.option = 0

        self.ctrl_cmd_type = CtrlCmd()
        self.ctrl_cmd_type.longlCmdType = 2

        self.proj_UTM = Proj(proj='utm', zone=52, ellps='WGS84', preserve_units=False)
        self.ego = EgoState()

        # self.ref_tracker = RefTracker(gain_lowspeed=Gain(Kp=2.2, Kd=0.2, Kpa=0.),
        #                               gain_highspeed=Gain(Kp=0.2, Kd=0.01, Kpa=0.1),
        #                               look_ahead_dist=0.7,
        #                               dt=1/50)
        self.ref_tracker=RefTracker()
        
        
        self.flag_ramp_stop = False 
        self.ramp_stop_time = 0

        self.dist_ahead = 0
        self.flag_dist_ahead_updated = False

        self.yolo_cls = -1
        self.lane_center = 0.0
        self.lane_filter = MovingAverageFilter(10)

        self.flag_emergency = False
        self.time_emergency = 0

    def main(self):
        loop_hz = 50
        rate = rospy.Rate(loop_hz)

        rospy.loginfo('morai_driver: waiting for gps topics')
        rospy.wait_for_message('/gps', GPSMessage)
        rospy.loginfo('morai_driver: begin')

        self.set_lamp(self.LAMP_LEFT)
        self.set_gear(self.GEAR_D)

        mission = self.MISSION_FIRST
        self.ref_tracker.set_ref_path('ref_course_1')
        self.ref_tracker.set_velocity_profile('velocity_profile_1')

        velocity = 0
        steering = 0

        l_ja_phase = 0
        nth_traffic_signal = 0
        parking_phase = 0
        

        while not rospy.is_shutdown():
            if self.check_emergency():
                continue
            if mission == self.MISSION_FIRST:
                if self.mission_first():
                    mission = self.MISSION_RAMP
            elif mission == self.MISSION_RAMP:
                if self.mission_ramp():
                    mission = self.MISSION_L_JA
            elif mission == self.MISSION_L_JA:
                if self.ego.x != 0:
                    steering, velocity, _= self.ref_tracker.do(self.ego)
                    self.set_cmd(steering, velocity)
                    if l_ja_phase == 4:
                        mission = self.MISSION_TRAFFIC_SIGNAL
                else:
                    if l_ja_phase == 0:
                        steering = 0
                        velocity = 3
                        if self.flag_dist_ahead_updated:
                            self.flag_dist_ahead_updated=False
                            if self.dist_ahead > 0.1 and self.dist_ahead < 5.3:
                                velocity=0
                                l_ja_phase = 1
                    elif l_ja_phase == 1:
                        steering = 1
                        velocity = 3
                        if self.ego.heading >= 6.09:
                            l_ja_phase = 2
                            steering=0
                            velocity=0
                    elif l_ja_phase == 2:
                        steering = 0
                        velocity = 3
                        if self.flag_dist_ahead_updated:
                            self.flag_dist_ahead_updated=False
                            if self.dist_ahead > 0.1 and self.dist_ahead < 5.3:
                                velocity=0
                                l_ja_phase = 3
                    elif l_ja_phase == 3:
                        steering=-1
                        velocity=3
                        if self.ego.heading <= 4.6:
                            steering=0
                            self.ref_tracker.set_ref_path('ref_course_2')
                            self.ref_tracker.set_velocity_profile('velocity_profile_2')
                            l_ja_phase=4
                    else:
                        velocity = 12

                    self.set_cmd(steering, velocity)
                    rospy.loginfo(f'{l_ja_phase}, {self.dist_ahead:0.1f}, {velocity:0.1f}, {self.ego.heading:0.4f}')

            elif mission == self.MISSION_TRAFFIC_SIGNAL:
                if self.mission_traffic_signal(nth_traffic_signal):
                    nth_traffic_signal += 1
                    if nth_traffic_signal == 1:
                        self.ref_tracker.set_ref_path('ref_course_3')
                        self.ref_tracker.set_velocity_profile('velocity_profile_3')
                        mission = self.MISSION_S_JA
                        self.first_s_ja=True
                    elif nth_traffic_signal == 2:
                        mission = self.MISSION_PARKING            
                    elif nth_traffic_signal == 3:
                        mission = self.MISSION_FINAL           
            
            elif mission == self.MISSION_PARKING:
                ret, parking_phase = self.mission_parking(parking_phase)
                if ret:
                    self.ref_tracker.set_ref_path('ref_course_4')
                    self.ref_tracker.set_velocity_profile('velocity_profile_4')
                    mission = self.MISSION_TRAFFIC_SIGNAL

            elif mission == self.MISSION_FINAL:
                steering, velocity, idx = self.ref_tracker.do(self.ego)
                self.set_cmd(steering, velocity)
                if idx > 2178:
                    self.set_lamp(self.LAMP_RIGHT)
                if idx > 2282:
                    self.set_cmd(0,0)
                    self.set_gear(self.GEAR_P)
                    mission = self.MISSION_END
                    rospy.logerr("FINISH!!!~~~")

            elif mission == self.MISSION_S_JA:
                if self.mission_s_ja():
                    mission = self.MISSION_TRAFFIC_SIGNAL

            elif mission == self.MISSION_END:
                self.set_cmd(0,0)

            rate.sleep()

    def mission_first(self):
        ret = False
        steering, velocity, idx = self.ref_tracker.do(self.ego)
        self.set_cmd(steering, velocity)
        if idx > 210: 
            self.set_lamp(self.LAMP_NO_SIGNAL)
            ret = True
        return ret
    
    def mission_ramp(self):
        ret = False
        steering, velocity, idx = self.ref_tracker.do(self.ego)

        if idx > 533 and idx < 553:
            if self.flag_ramp_stop == False:
                self.set_cmd(0, 0)
                self.flag_ramp_stop = True
                self.ramp_stop_time = time.time()
            else:
                if time.time() - self.ramp_stop_time > 3.2:
                    ret = True
        else:
            self.set_cmd(steering, velocity)
        
        return ret
    
    def mission_traffic_signal(self, nth_traffic_signal):
        ret =False
        steering, velocity, idx = self.ref_tracker.do(self.ego)
        if nth_traffic_signal == 0:
            traffic_signal_idx_min, traffic_signal_idx_max = 260, 275
            if idx > 810:
                ret = True
        elif nth_traffic_signal == 1:
            self.observe_gorani(idx)
            traffic_signal_idx_min, traffic_signal_idx_max = 204, 210            
            if self.dist_ahead > 0.1 and self.dist_ahead < 5.5 and idx > 1200:
                velocity = 0
                ret = True
        elif nth_traffic_signal == 2:
            traffic_signal_idx_min, traffic_signal_idx_max = 335,350
            if idx > 500:
                ret = True

        if idx >= traffic_signal_idx_min and idx <= traffic_signal_idx_max:
            if self.yolo_cls == 0:
                velocity = 0

        self.set_cmd(steering, velocity)

        return ret
    
    def mission_s_ja(self):
        if self.first_s_ja:
            if self.ego.heading < 6.2:
                self.set_cmd(-1,2)
                rospy.loginfo(f"turn right {self.ego.heading}")
                return
            else:
                self.first_s_ja = False
        self.lane_filter.update(self.lane_center)

        e = 320-self.lane_filter.get_moving_average()
        Kp = 0.015
        steering = Kp * e
        velocity = 7
        self.set_cmd(steering,velocity)

        if self.ego.x != 0:
            return True
        
        rospy.loginfo(f'e: {e:0.1f}, y: {self.lane_center:0.1f}')
        return False


    def mission_parking(self, parking_phase):
        if parking_phase == 0:
            if self.ego.heading >= 5.71:
                rospy.logwarn(f"turn right {self.ego.heading:0.3f}")
                steering = -1
                velocity = 2
                self.set_cmd(steering, velocity)
            else:
                parking_phase = 1
        elif parking_phase == 1:
            rospy.logwarn("stop")
            self.set_cmd(0,0)
            rospy.logwarn("set ger R")
            self.set_gear(self.GEAR_R)
            parking_phase = 2
        elif parking_phase == 2:
            rospy.logwarn(f"turn left {self.ego.heading:0.3f}")
            steering = 1
            velocity = 3
            self.set_cmd(steering, velocity)
            if self.ego.heading < 4.556:
                parking_phase = 3
        elif parking_phase == 3:
            rospy.logwarn(f"back {self.dist_ahead}")
            steering = 0
            velocity = 2
            self.set_cmd(steering, velocity)
            if self.dist_ahead >= 6.54:
                parking_phase = 4
                self.set_cmd(0,0)
                self.set_gear(self.GEAR_D)
        elif parking_phase == 4:
            rospy.logwarn(f"forward {self.dist_ahead}")
            steering = 0
            velocity = 2
            self.set_cmd(steering, velocity)
            if self.dist_ahead > 0.1 and self.dist_ahead < 4.99:
                parking_phase = 5
        elif parking_phase == 5:
            rospy.logwarn(f"turn right {self.ego.heading:0.3f}")
            steering = -1
            velocity = 2
            self.set_cmd(steering, velocity)
            if self.ego.heading < 3:
                parking_phase = 6
        else:
            return True, parking_phase
        
        return False, parking_phase

    def observe_gorani(self, idx):
        if self.dist_ahead > 0.1 and self.dist_ahead < 8:
            if idx >= 120 and idx <= 180 or \
               idx >= 371 and idx <= 508 or \
               idx >= 790 and idx <= 915:
                self.flag_emergency = True
                self.time_emergency = time.time()

    def check_emergency(self):
        if self.flag_emergency:
            if time.time() - self.time_emergency > 5 and self.dist_ahead == 0:
                rospy.logerr("GO")
                self.flag_emergency=False
                self.set_lamp(self.LAMP_NO_SIGNAL)
                return False
            else:
                rospy.logerr("EMERGENCY")
                velocity=0
                steering = 0
                self.set_lamp(self.LAMP_EMERGENCY)
                self.set_cmd(steering, velocity)
                return True


    def set_cmd(self, steer, vel):
        self.ctrl_cmd_type.velocity=vel
        self.ctrl_cmd_type.steering=steer
        self.pub_ctrl_cmd.publish(self.ctrl_cmd_type)

    def cb_gps(self, data):
        if data.longitude != 0:
            self.ego.x, self.ego.y = self.proj_UTM(data.longitude, data.latitude)
        else:
            self.ego.x, self.ego.y, self.ego.velocity = 0, 0, 0

    def cb_velocity(self, data):
        self.ego.velocity = data.data

    def cb_imu(self, data):
        euler = euler_from_quaternion([data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w])
        self.ego.heading = euler[2]
        self.ego.heading = 2*np.pi + self.ego.heading if self.ego.heading < 0 else self.ego.heading

    def cb_dist_ahead(self, data):
        self.dist_ahead = data.data
        self.flag_dist_ahead_updated=True

    def cb_yolo_cls(self, data):
        self.yolo_cls = data.data

    def cb_lane(self, data):
        self.lane_center = data.data

    def set_lamp(self, signal_type):
        lamps = Lamps()
        if signal_type == self.LAMP_NO_SIGNAL:
            lamps.turnSignal = 0
            lamps.emergencySignal = 0
        elif signal_type == self.LAMP_LEFT:
            lamps.turnSignal = 1
            lamps.emergencySignal = 0
        elif signal_type == self.LAMP_RIGHT:
            lamps.turnSignal = 2
            lamps.emergencySignal = 0
        elif signal_type == self.LAMP_EMERGENCY:
            lamps.turnSignal = 0
            lamps.emergencySignal = 1

        self.event_cmd_srv_type.option = 4
        self.event_cmd_srv_type.lamps = lamps
        _ = self.srv_event_cmd(self.event_cmd_srv_type)
    
    def set_gear(self, gear):
        while abs(self.ego.velocity > 1):
            continue

        self.event_cmd_srv_type.option = 2
        self.event_cmd_srv_type.gear = gear
        _ = self.srv_event_cmd(self.event_cmd_srv_type)
        
if __name__ == '__main__':
    morai_driver = MoraiDriver()
    morai_driver.main()

