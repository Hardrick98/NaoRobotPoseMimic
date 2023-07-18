#!/usr/bin/env python3

import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from moveit_commander.conversions import pose_to_list

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
        rospy.init_node('nao_move_config', anonymous=True)

        robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()

        group_name = "right_arm"
        move_group = moveit_commander.MoveGroupCommander(group_name)

       
        planning_frame = move_group.get_planning_frame()
        end_effector_link = move_group.get_end_effector_link()
        group_names = robot.get_group_names()

        self.box_name = ''
        self.robot = robot
        self.scene = scene
        self.move_group = move_group
        self.planning_frame = planning_frame
        self.end_effector_link = end_effector_link
        self.group_names = group_names


    def go_to_pose_goal(self):
        print("Starting planning to go to pose goal.\n")

        
        """
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.orientation.w = 1.0
        pose_goal.orientation.x = 0.006
        pose_goal.orientation.y = 0.003
        pose_goal.orientation.z = -0.018
        pose_goal.position.x = 0.5 #0.112
        pose_goal.position.y = 0.3 #-0-093
        pose_goal.position.z = 3.5 #0.195      
    
        success = self.move_group.go(wait=True)

        self.move_group.stop()

        self.move_group.clear_pose_targets()
        
        print(pose_goal)
        """
        joint_goal = self.move_group.get_current_joint_values()
        joint_goal[0] = 0 #spalla avanti-indietro [-1.7,1.7]
        joint_goal[1] = -1.8 #spalla rotazione destra-sinistra
        joint_goal[2] = 0 #rotazione gomito
        joint_goal[3] = -1.4 #spostamento x-y gomito
        joint_goal[4] = 0  #rotazione polso
        


	
        self.move_group.go(joint_goal)
        self.move_group.stop()

        self.move_group.clear_pose_targets()
        
        # For testing:  
        current_pose = self.move_group.get_current_pose().pose
        print(current_pose)
        

def main():
    moveit_context = MoveItContext()

    moveit_context.go_to_pose_goal()
    



if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
