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

        image = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        image = cv2.convertScaleAbs(image, alpha=(255.0 / np.max(image)))
    except CvBridgeError as e:
         print(e)
    
    name = "/home/rick/Immagini/depth" + str(count) + ".jpg"
    
    cv2.imwrite(name, image)
    cv2.imshow("stream",image)
    cv2.waitKey(1)
    # Access the image using msg.data, msg.width, msg.height, etc.
    # Perform any desired image processing or analysis
    count += 1
    #result = model(image)

    #print(result)

    pass

rospy.init_node('depth_node')
rospy.Subscriber('/naoqi_driver/camera/ir/image_raw', Image, image_callback)
rospy.spin()