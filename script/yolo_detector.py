#!/usr/bin/env python3
# -*- coding: utf-8 -*- 16

import os
import rospy
import rospkg
from ultralytics import YOLO
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Int16
from cv_bridge import CvBridge

class YoloDetector():
    def __init__(self, weight_file, topic_name):
        rospy.init_node('yolo_detector')
        self.yolo = YOLO(weight_file)
        self.bridge = CvBridge()
        self.topic_name = topic_name
        
        rospy.Subscriber(topic_name,CompressedImage,self.cb_cam,queue_size=1)
        self.pub_yolo_img = rospy.Publisher('/yolo_img', Image, queue_size=1)
        self.pub_yolo_cls = rospy.Publisher('/yolo_cls', Int16, queue_size=1)

    def main(self):
        loop_hz = 10
        rate = rospy.Rate(loop_hz)
        rospy.logwarn('yolo_detector: waiting for cam topic')
        rospy.wait_for_message(self.topic_name,CompressedImage)
        rospy.logwarn('yolo_detector: begin')
        while not rospy.is_shutdown():
            result = self.yolo(self.img, verbose=False)
            boxes = result[0].boxes
            num_boxes = len(boxes)
            if num_boxes != 0:
                rospy.loginfo(f'num_boxes: {num_boxes}, cls: {boxes.cls[0]}')
                self.pub_yolo_img.publish(self.bridge.cv2_to_imgmsg(result[0].plot(),encoding='bgr8'))
                self.pub_yolo_cls.publish(int(boxes.cls[0]))
            rate.sleep()

    def cb_cam(self, msg):
        self.img = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="bgr8")


if __name__ == '__main__':
    rospack = rospkg.RosPack()
    weight_file = os.path.join(rospack.get_path('dku_morai_driver'), 
                                                'weight/best_s.pt')
    yd = YoloDetector(weight_file=weight_file, 
                      topic_name='/image_traffic_light/compressed')
    yd.main()