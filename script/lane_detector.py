#!/usr/bin/env python3
# -*- coding: utf-8 -*- 16
import numpy as np
import rospy
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Float32
from cv_bridge import CvBridge
from collections import deque
import cv2
from BEV import BEVTransform
import rospkg
import os
import json
from finding_lines import Line, find_LR_lines, draw_lane, print_road_status, lane_median, draw_follower


class LaneDetector():
    def __init__(self, topic_name):
        self.bridge = CvBridge()

        rospy.init_node('lane_detector')
        rospy.Subscriber(topic_name, CompressedImage, self.cb_cam, queue_size=1)
        self.pub_lane_center = rospy.Publisher('/lane_center', Float32, queue_size=1)

        rp = rospkg.RosPack()
        currentPath = rp.get_path("kimdom")
        with open(os.path.join(currentPath, 'sensor/sensor_params.json'), 'r') as fp:
            sensor_params = json.load(fp)
        
        self.bev = BEVTransform(params_cam = sensor_params["params_cam"])
        self.left_line = Line()
        self.right_line = Line()


        rospy.logwarn('lane_detector: wait for lane image')
        rospy.wait_for_message(topic_name, CompressedImage)
        rospy.logwarn('lane_detector: begin')
                

    def main(self):
        loop_hz = 20
        rate = rospy.Rate(loop_hz)
       
        while not rospy.is_shutdown():
            img = self.img.copy()
            result = self.detect(img)
            cv2.imshow("res", result)
            cv2.waitKey(1)
            rate.sleep()

    def detect_edge(self, hsv):
        #orange lane (yellow)
        lower_orange_lane = (20, 200, 200)
        upper_orange_lane = (40, 255, 255)
        mask_orange_lane = cv2.inRange(hsv, lower_orange_lane, upper_orange_lane)

        mask = mask_orange_lane
        return mask
    

    def detect(self, undist_img):
        img_warp = cv2.GaussianBlur(self.bev.warp_bev_img(undist_img), (5, 5), 0)
        combined_hsv = cv2.cvtColor(img_warp, cv2.COLOR_BGR2HSV)
        
        s_val = combined_hsv[:,:,1]
        canny_edge = cv2.Canny(s_val, 100, 200)

        hsv_orange = self.detect_edge(combined_hsv)

        edgeandhsv = cv2.addWeighted(hsv_orange, 1, canny_edge, 1, 0)

        searching_img = find_LR_lines(edgeandhsv, self.left_line, self.right_line)

        if searching_img.shape[0] == 0:
            self.left_line = Line()
            self.right_line = Line()
            rospy.logerr("lane detection fail")
            result = undist_img
        else:
            _, w_color_result = draw_lane(searching_img, self.left_line, self.right_line)
            median = lane_median(self.left_line, self.right_line)
            
            self.pub_lane_center.publish(median)
            median_line = draw_follower(median, w_color_result)

            color_result = self.bev.warp_inv_img(median_line)

            result = cv2.addWeighted(undist_img, 1, color_result, 0.3, 0)
            info = np.zeros_like(result)
            info[5:110, 5:190] = (255, 255, 255)
            info = cv2.addWeighted(result, 1, info, 0.5, 0)
            info = print_road_status(info, self.left_line, self.right_line)
            info = cv2.addWeighted(result, 0.5, info, 0.3, 0)

        return info
    
    
    def cb_cam(self, msg):
        self.img = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="bgr8")


if __name__ == '__main__':
    topic_name = '/image_lane/compressed'
    ld = LaneDetector(topic_name=topic_name)
    ld.main()