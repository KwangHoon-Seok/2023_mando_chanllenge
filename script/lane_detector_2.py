#!/usr/bin/env python3
# -*- coding: utf-8 -*- 16
import numpy as np
import rospy
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Float32
from cv_bridge import CvBridge
from collections import deque
import cv2

class LaneDetector():
    def __init__(self, topic_name):
        self.bridge = CvBridge()

        rospy.init_node('lane_detector')
        rospy.Subscriber(topic_name, CompressedImage, self.cb_cam, queue_size=1)
        self.pub_lane_center = rospy.Publisher('/lane_center', Float32, queue_size=1)

        rospy.logwarn('lane_detector: wait for lane image')
        rospy.wait_for_message(topic_name, CompressedImage)
        rospy.logwarn('lane_detector: begin')
                
        # canny params
        self.canny_low, self.canny_high = 50, 80

        # HoughLineP params
        self.hough_threshold, self.min_length, self.min_gap = 40, 50, 10

        # perspective config
        self.img_size = (640, 480)
        self.warp_img_w, self.warp_img_h, self.warp_img_mid = 640, 200, 100

        # initial state
        self.angle = 0.0
        self.prev_angle = deque([0.0], maxlen=5)
        self.lane = np.array([90.0, 320., 568.])

        # filtering params:
        self.angle_tolerance = np.radians(30)
        self.cluster_threshold = 200

        src_pts = np.float32([[133, 289], 
                              [481, 290], 
                              [640, 350], 
                              [0, 350]])
        dst_pts = np.float32([[0,0], 
                              [640,0], 
                              [640,200], 
                              [0,200]])
        self.warp_src  = np.array(src_pts, dtype=np.float32)
        self.warp_dist = np.array(dst_pts, dtype=np.float32)
        self.M = cv2.getPerspectiveTransform(self.warp_src, self.warp_dist)
    
        self.debug = False

    def main(self):
        loop_hz = 20
        rate = rospy.Rate(loop_hz)
       
        while not rospy.is_shutdown():
            img = self.img.copy()
            self.detect(img)
            cv2.waitKey(1)
            rate.sleep()

    def cb_cam(self, msg):
        self.img = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="bgr8")


    def to_perspective(self, img):
        img = cv2.warpPerspective(img, self.M, (self.warp_img_w, self.warp_img_h), flags=cv2.INTER_LINEAR)
        if self.debug:
            cv2.imshow('bird-eye-view', img)
        return img

    def to_canny(self, img):
        img = cv2.GaussianBlur(img, (7,7), 0)
        img = cv2.Canny(img, self.canny_low, self.canny_high)
        if self.debug:
            cv2.imshow('canny', img)
        return img

    def hough_transform(self, img):
        lines = cv2.HoughLinesP(img, 1, np.pi/180, self.hough_threshold, self.min_gap, self.min_length)
        if self.debug:
            hough_img = np.zeros((img.shape[0], img.shape[1], 3))
            if lines is not None:
                for x1, y1, x2, y2 in lines[:, 0]:
                    cv2.line(hough_img, (x1, y1), (x2, y2), (0,0,255), 2)
            cv2.imshow('hough', hough_img)
        return lines

    
    def detect_yellow(self, img):    
        #orange lane (yellow)
        hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        lower_orange_lane = (20, 200, 200)
        upper_orange_lane = (40, 255, 255)
        mask_orange_lane = cv2.inRange(hsv_img, lower_orange_lane, upper_orange_lane)

        mask = mask_orange_lane
        if self.debug:
            cv2.imshow('mask', mask)
        return mask

    def filter(self, img):
        '''
        filter lines that are close to previous angle and calculate its positions
        '''
        thetas, positions = [], []
        if self.debug:
            filter_img = np.zeros((self.warp_img_h, self.warp_img_w, 3))

        if img is not None:
            for x1, y1, x2, y2 in img[:, 0]:
                if y1 == y2:
                    continue
                flag = 1 if y1-y2 > 0 else -1
                theta = np.arctan2(flag * (x2-x1), flag * 0.9* (y1-y2))
                if abs(theta - self.angle) < self.angle_tolerance:
                    position = float((x2-x1)*(self.warp_img_mid-y1))/(y2-y1) + x1
                    thetas.append(theta)
                    positions.append(position) 
                    if self.debug:
                        cv2.line(filter_img, (x1, y1), (x2, y2), (0,0,255), 2)

        self.prev_angle.append(self.angle)
        if thetas:
            self.angle = np.mean(thetas)
        if self.debug:
            cv2.imshow('filtered lines', filter_img)
        return positions


    def get_cluster(self, positions):
        '''
        group positions that are close to each other
        '''
        clusters = []
        for position in positions:
            if 0 <= position < self.warp_img_w:
                for cluster in clusters:
                    if abs(cluster[0] - position) < self.cluster_threshold:
                        cluster.append(position)
                        break
                else:
                    clusters.append([position])
        lane_candidates = sorted([np.mean(cluster) for cluster in clusters])
        return lane_candidates
    
   

    def mark_lane(self, img, lane=None):
        '''
        mark calculated lane position to an image 
        '''
        img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
        if lane is None:
            lane = self.lane
        l1, l2, l3 = self.lane
        cv2.circle(img, (int(l1), self.warp_img_mid), 3, (0,0,255), 5, cv2.FILLED)
        cv2.circle(img, (int(l2), self.warp_img_mid), 3, (0,255,0), 5, cv2.FILLED)
        cv2.circle(img, (int(l3), self.warp_img_mid), 3, (255,0,0), 5, cv2.FILLED)
        cv2.imshow('marked', img)


    def detect(self, img):
        '''
        returns angle and cte of a target lane from an image
        angle : radians
        cte : pixels
        '''
        yellow_img = self.detect_yellow(img)
        
        canny_img = self.to_canny(yellow_img)
        img_perspective = self.to_perspective(canny_img)
        img_hough = self.hough_transform(img_perspective)
        positions = self.filter(img_hough)
        lane_candidates = self.get_cluster(positions)        
        if len(lane_candidates) == 1:
            if lane_candidates[0] < 320:
                self.lane[0] = lane_candidates[0]
                self.lane[1] = lane_candidates[0] + 270
                self.lane[2] = lane_candidates[0] + 560
            else:
                self.lane[0] = lane_candidates[0] - 560
                self.lane[1] = lane_candidates[0] - 270
                self.lane[2] = lane_candidates[0]
        elif len(lane_candidates)==2:
            if abs(lane_candidates[1]-lane_candidates[0]) >= 500:
                self.lane[0] = lane_candidates[0]
                self.lane[1] = np.mean(lane_candidates)
                self.lane[2] = lane_candidates[1]
        self.mark_lane(img_perspective)

        self.pub_lane_center.publish(self.lane[1])
        
if __name__ == '__main__':
    topic_name = '/image_lane/compressed'
    ld = LaneDetector(topic_name=topic_name)
    ld.main()