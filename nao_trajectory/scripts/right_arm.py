#!/usr/bin/env python3
from moveit_msgs.msg import RobotState, Constraints, OrientationConstraint
import sys
import rospy
import moveit_commander
import geometry_msgs.msg
import numpy as np
from scipy.spatial.transform import Rotation as R
from moveit_commander.conversions import pose_to_list
from std_msgs.msg import Float32MultiArray

def all_close(goal, actual, tolerance):
    """
    Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
    @param: goal       A list of floats, a Pose or a PoseStamped
    @param: actual     A list of floats, a Pose or a PoseStamped
    @param: tolerance  A float
    @returns: bool
    """
    all_equal = True
    if type(goal) is list:
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False

    elif type(goal) is geometry_msgs.msg.PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)

    elif type(goal) is geometry_msgs.msg.Pose:
        return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

    return True

class MoveItContext(object):
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        

        robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()

        group_name = "right_arm"
        move_group = moveit_commander.MoveGroupCommander(group_name)
    

       
        planning_frame = move_group.get_planning_frame()
        end_effector_link = move_group.get_end_effector_link()
        print(end_effector_link)
        group_names = robot.get_group_names()

        self.box_name = ''
        self.robot = robot
        self.scene = scene
        self.received_pose = None

        self.move_group = move_group
        self.planning_frame = planning_frame
        self.end_effector_link = end_effector_link
        self.group_names = group_names

        self.current_pose = self.move_group.get_current_pose().pose


    def set_received_pose(self, pose_msg):

        self.received_pose = pose_msg
            
        

    def go_to_pose_goal(self, pose_msg):
        print("Starting planning to go to pose goal.\n")

        self.move_group.set_workspace([-10,-10,0,10,10,10])
        self.move_group.set_start_state_to_current_state() 	
        self.move_group.clear_path_constraints() 	
        print(self.move_group.get_planning_frame())
        
        rhand = pose_msg
        
        x = -rhand[2]/0.50*0.14 + 0.02
        z = -rhand[1]/0.80*0.32-0.06
        y = rhand[0]/0.50*0.17 - 0.02

        print(x,y,z)

        if x > 0.16:

            x = 0.16

        if x < 0.07:

            x = 0.07
        
        if y < -0.20:

            y = -0.20

        if y > -0.02:

            y = -0.02

        if z>0.21:
            
            z = 0.21

        if z<-0.047:
            
            z = -0.047
            y = -0.144
            x = 0.033


        print(x,y,z)

        pose_goal = geometry_msgs.msg.Pose()

        pose_goal.position.x = float(x)
        pose_goal.position.y = float(y)
        pose_goal.position.z = float(z)

        self.move_group.set_position_target([pose_goal.position.x, pose_goal.position.y, pose_goal.position.z])
    

        if (x<0.160 and x >= 0):

            self.move_group.set_start_state_to_current_state()
            
            plan = self.move_group.plan()
            self.move_group.execute(plan[1], wait=True)
            

            self.move_group.clear_pose_targets()
          

def pose_callback(pose_msg):

    moveit_context = MoveItContext()
    moveit_context.go_to_pose_goal(pose_msg.data)


    



if __name__ == '__main__':
    rospy.init_node('nao_move_config', anonymous=True)
    try:
        rospy.Subscriber('rhand_pose_topic', Float32MultiArray, pose_callback)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
