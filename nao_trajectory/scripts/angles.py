#!/usr/bin/env python3
from moveit_msgs.msg import RobotState, Constraints, OrientationConstraint
import sys
import rospy
import numpy as np
from scipy.spatial.transform import Rotation as R
from moveit_commander.conversions import pose_to_list
from std_msgs.msg import Float32MultiArray

from conversion import convert_to_quaternion




def pose_callback(pose_msg):
        
        print("Starting planning to go to pose goal.\n")

    
        
        pose = pose_msg.data

            
        lhand = pose
        
        x = -lhand[2]/2.2
        z = -54*lhand[1]/160
        y = lhand[0]/2
    
        
        print(x)
        print(y)
        roll = 0
        pitch = x / 0.320 * 3.2 - 1.6

        yaw = y / 0.320* 3.2 - 1.6

        quaternion = convert_to_quaternion(roll, pitch, yaw)

        print(quaternion)

        

            




if __name__ == '__main__':
    rospy.init_node('orientation_node', anonymous=True)
    try:
        rospy.Subscriber('lhand_pose_topic', Float32MultiArray, pose_callback)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
