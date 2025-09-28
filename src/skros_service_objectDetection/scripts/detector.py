#!/usr/bin/env python3
# (http://wiki.ros.org/ROS/Tutorials/WritingServiceClient%28python%29)

# ObjectClassificationServer packages

import os
from pickle import FALSE, TRUE
import sys
import rospy
import ros_utils

import numpy as np

import sensor_msgs.msg
from std_msgs.msg import Float32, Bool

import torch
import torch.nn as nn
import cv2
from PIL import Image

class objectDetector:
    def __init__(self):
        # load model
        self.model = torch.hub.load('.', 'custom', path='./weights/yolo5s_best.pt', source='local', device='cpu', force_reload=True)
        for m in self.model.modules():
            if isinstance(m, nn.Upsample):
                m.recompute_scale_factor = None

        # include pose subscriber
        #rospy.Subscriber("/skros_device_camera_realsense/color/image_raw", sensor_msgs.msg.Image, self.callback, queue_size=1)
        rospy.Subscriber("/cam1/skros_device_camera_realsense/color/image_raw", sensor_msgs.msg.Image, self.callback, queue_size=1)
        
        # include pose publisher
        self.pub_door_detection_confidence_topic = rospy.Publisher("/door_detection_confidence", Float32, queue_size=1)
        self.pub_door_available_topic = rospy.Publisher("/door_available", Bool, queue_size=1)
        self.message_conf = Float32()
        self.message_bool = Bool()
        self.cropped_image = []
        self.width = 640
        self.height = 480


    def callback(self, image):
        
        np_image = ros_utils.image2numpy(image)
        np_image = cv2.resize(np_image, (self.width, self.height), interpolation=cv2.INTER_NEAREST)
        #print (np_image.shape)
        results = self.model(np_image)
        results_var = results.pandas().xyxy[0]
        
        num_objects = results_var.shape[0]
        #print(num_objects)
        # Results, change the flowing to: results.show()
        #results.show()  # or .show(), .save(), .crop(), .pandas(), etc
        #results.print()  # or .show(), .save()

        if num_objects == 0:
            self.message_conf.data = 0
            self.pub_door_detection_confidence_topic.publish(self.message_conf)
            self.message_bool.data = False
            self.pub_door_available_topic.publish(self.message_bool)
        else:
            print(results_var)
            max_confi = 0.0
            
            for i in range(num_objects):                
                x0,y0,x1,y1,confi,cla = results.xyxy[0][i].numpy()                
                if cla == 0:
                    #print(x0,y0,x1,y1,confi,cla)
                    if confi > max_confi:
                        max_confi = confi
                    
            if max_confi > 0.4:
                door_available = True
            else:
                door_available = False
            
            self.message_conf.data = max_confi
            self.pub_door_detection_confidence_topic.publish(self.message_conf)
            self.message_bool.data = door_available
            self.pub_door_available_topic.publish(self.message_bool)
          
def main(args):
    
    rospy.init_node("objectDetector", anonymous = False)
    ic = objectDetector()
    try:
        rospy.loginfo('objectDetector is now available.')
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == '__main__':
     main(sys.argv)

