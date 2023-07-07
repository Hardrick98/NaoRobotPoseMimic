#! /usr/bin/env python3

import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import message_filters
import cv2
import numpy as np

def read_cameras():
    
    imageL = message_filters.Subscriber("/naoqi_driver/camera/front/image_raw", Image)
    imageR = message_filters.Subscriber("/naoqi_driver/camera/ir/image_raw", Image)

    # Synchronize images
    ts = message_filters.ApproximateTimeSynchronizer([imageL, imageR], queue_size=10, slop=1)
    ts.registerCallback(image_callback)
    rospy.spin()

def image_callback(imageL, imageR):
    global count
    br = CvBridge()
    rospy.loginfo("receiving frame")
    imageRGB = br.imgmsg_to_cv2(imageL, desired_encoding='bgr8')

    image = br.imgmsg_to_cv2(imageR, desired_encoding='passthrough')
    imageIR = cv2.convertScaleAbs(image, alpha=(255.0 / np.max(image)))   

    name_rgb = "/home/rick/Immagini/stereo/rgb" + str(count) + ".jpg"
    cv2.imwrite(name_rgb, imageRGB)
  
    

    name_ir = "/home/rick/Immagini/stereo/ir" + str(count) + ".jpg"
    cv2.imwrite(name_ir, imageIR)
   
    count += 1

if __name__ == '__main__':
    rospy.init_node('stereo_node')
    count = 0
    try:
        read_cameras()
        
    except rospy.ROSInterruptException:
        pass
