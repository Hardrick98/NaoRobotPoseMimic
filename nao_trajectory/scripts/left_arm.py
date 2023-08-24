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

        group_name = "left_arm"
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

        """
        self.upright_constraints = Constraints()
        self.upright_constraints.name = "upright"
        orientation_constraint = OrientationConstraint()
        orientation_constraint.link_name = self.move_group.get_end_effector_link()
        orientation_constraint.orientation = self.current_pose.orientation
        orientation_constraint.header.frame_id = "l_wrist"
        orientation_constraint.absolute_x_axis_tolerance = 3.14
        orientation_constraint.absolute_y_axis_tolerance = 3.14
        orientation_constraint.absolute_z_axis_tolerance = 3.14
        #orientation_constraint.absolute_z_axis_tolerance = 3.14 #ignore this axis
        orientation_constraint.weight = 1

        self.upright_constraints.orientation_constraints.append(orientation_constraint)
        """

    def set_received_pose(self, pose_msg):

        self.received_pose = pose_msg
            
        

    def go_to_pose_goal(self, pose_msg):
        print("Starting planning to go to pose goal.\n")

        self.move_group.set_workspace([-10,-10,0,10,10,10])
        self.move_group.set_start_state_to_current_state() 	
        self.move_group.clear_path_constraints() 	
        print(self.move_group.get_planning_frame())
        #print(self.upright_constraints)
        #self.move_group.set_path_constraints(self.upright_constraints)
        
        
        #rhand = pose[16]

        lhand = pose_msg
        
        x = -lhand[2]/2.2
        z = -54*lhand[1]/160
        y = lhand[0]/2

        print(x,y,z)

    
        pose_goal = geometry_msgs.msg.Pose()

        pose_goal.position.x = float(x)
        pose_goal.position.y = float(y)
        pose_goal.position.z = float(z)

        self.move_group.set_position_target([pose_goal.position.x, pose_goal.position.y, pose_goal.position.z])
    #0.195  


        if (x<0.160 and x >= 0):

            self.move_group.set_start_state_to_current_state()
            
            plan = self.move_group.plan()
            self.move_group.execute(plan[1], wait=True)
            
           
        
            #success = self.move_group.go(wait=True)

            #self.move_group.stop()

            #print(success)

            self.move_group.clear_pose_targets()
          

def pose_callback(pose_msg):

    moveit_context = MoveItContext()
    moveit_context.go_to_pose_goal(pose_msg.data)
    # This callback function will be called whenever a new pose is received


    



if __name__ == '__main__':
    rospy.init_node('nao_move_config', anonymous=True)
    try:
        rospy.Subscriber('lhand_pose_topic', Float32MultiArray, pose_callback)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
