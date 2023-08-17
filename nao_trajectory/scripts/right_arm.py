#!/usr/bin/env python3
from moveit_msgs.msg import RobotState, Constraints, OrientationConstraint
import sys
import rospy
import moveit_commander
import geometry_msgs.msg
import numpy as np
from scipy.spatial.transform import Rotation as R
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
        print(end_effector_link)
        group_names = robot.get_group_names()

        self.box_name = ''
        self.robot = robot
        self.scene = scene
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


    def go_to_pose_goal(self):
        print("Starting planning to go to pose goal.\n")

        self.move_group.set_workspace([-10,-10,0,10,10,10])
        self.move_group.set_start_state_to_current_state() 	
        self.move_group.clear_path_constraints() 	
        print(self.move_group.get_planning_frame())
        #print(self.upright_constraints)
        #self.move_group.set_path_constraints(self.upright_constraints)
        
        data  = np.load('/home/rick/VideoPose3D/joints.npy')
        lst = data

        for i in range(int(len(lst)/25)):
 	

            pose = lst[i*25]

            rhand = pose[16]

            #lhand = pose[13]
            
            x = -rhand[2]/2.2
            z = -54*rhand[1]/160
            y = rhand[0]/2

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
            
            """
            joint_goal = self.move_group.get_current_joint_values()
            joint_goal[0] = 0 #spalla avanti-indietro [-1.7,1.7]
            joint_goal[1] = 0 #spalla rotazione destra-sinistra
            joint_goal[2] = 0 #rotazione gomito
            joint_goal[3] = 0 #spostamento x-y gomito  
            joint_goal[4] = 0 #rotazione polso
            """

            

def main():
    moveit_context = MoveItContext()

    moveit_context.go_to_pose_goal()
    



if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
