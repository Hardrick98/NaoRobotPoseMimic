#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
import torch

#model = torch.hub.load('ultralytics/yolov5', 'yolov5s', pretrained=True)

bridge = CvBridge()

count = 0

def image_callback(msg):

    global count

    
    try:

        image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    
    except CvBridgeError as e:
         print(e)
    
    name = "/home/rick/Immagini/rgb" + str(count) + ".jpg"
    
    cv2.imwrite(name, image)
    cv2.imshow("stream",image)
    cv2.waitKey(1)
    # Access the image using msg.data, msg.width, msg.height, etc.
    # Perform any desired image processing or analysis
    count += 1
    #result = model(image)

    #print(result)

    pass

rospy.init_node('rgb_node')
rospy.Subscriber('/naoqi_driver/camera/front/image_raw', Image, image_callback)
rospy.spin()
