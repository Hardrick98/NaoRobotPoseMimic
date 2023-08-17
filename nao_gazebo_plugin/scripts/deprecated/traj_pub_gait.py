#!/usr/bin/env python3
import rospy
from math import pi
from std_msgs.msg import Header
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from helpers import generate_leg_positions


def pub_joint_states(event):
    t = event.current_real.to_sec()
    # Generating the trajectory messages for the controllers

    xr, zr, xl, zl = generate_leg_positions(t)


    # todo: import IK and use that function here to get joint positions
    #  then use that to update the joint positions for ROS
    # right_leg_joint_positions = IK(xr,0,zr)
    # left_leg_joint_positions = IK(xl,0,zl)

    # todo: this is placeholder.  Update to output from IK when available.
    l_hip_roll = 0
    l_hip_pitch = 0
    l_knee_pitch = 0
    r_hip_roll = 0
    r_hip_pitch = 0
    r_knee_pitch = 0
    l_ankle_pitch_joint = 0
    l_ankle_roll = 0
    r_ankle_pitch_joint = 0
    r_ankle_roll = 0

    # Publish updated joint positions for the legs
    # Everything else will remain constant

    # Left Arm
    la = JointTrajectory()
    la.joint_names = ['LShoulderPitch', 'LShoulderRoll', 'LElbowYaw', 'LElbowRoll', 'LWristYaw']
    lapt = JointTrajectoryPoint()
    lapt.positions = [pi/2, 0, 0, 0, 0]
    lapt.time_from_start = rospy.Duration.from_sec(1)
    la.points.append(lapt)

    # Right Arm
    ra = JointTrajectory()
    ra.joint_names = ['RShoulderPitch', 'RShoulderRoll', 'RElbowYaw', 'RElbowRoll', 'RWristYaw']
    rapt = JointTrajectoryPoint()
    rapt.positions = [pi/2, 0, 0, 0, 0]
    rapt.time_from_start = rospy.Duration.from_sec(1)
    ra.points.append(rapt)

    # Left Hand
    lh = JointTrajectory()
    lh.joint_names = ['LHand']
    lhpt = JointTrajectoryPoint()
    lhpt.positions = [0]
    lhpt.time_from_start = rospy.Duration.from_sec(1)
    lh.points.append(lhpt)

    # Right Hand
    rh = JointTrajectory()
    rh.joint_names = ['RHand']
    rhpt = JointTrajectoryPoint()
    rhpt.positions = [0]
    rhpt.time_from_start = rospy.Duration.from_sec(1)
    rh.points.append(rhpt)

    # Pelvis
    p = JointTrajectory()
    p.joint_names = ['LHipYawPitch']
    ppt = JointTrajectoryPoint()
    ppt.positions = [0]
    ppt.time_from_start = rospy.Duration.from_sec(1)
    p.points.append(ppt)

    # Left Leg
    ll = JointTrajectory()
    ll.joint_names = ['LHipRoll', 'LHipPitch', 'LKneePitch']
    llpt = JointTrajectoryPoint()
    llpt.positions = [l_hip_roll, l_hip_pitch, l_knee_pitch]
    llpt.time_from_start = rospy.Duration.from_sec(1)
    ll.points.append(llpt)    
    
    # Right Leg
    rl = JointTrajectory()
    rl.joint_names = ['RHipRoll', 'RHipPitch', 'RKneePitch']
    rlpt = JointTrajectoryPoint()
    rlpt.positions = [r_hip_roll, r_hip_pitch, r_knee_pitch]
    rlpt.time_from_start = rospy.Duration.from_sec(1)
    rl.points.append(rlpt)    
    
    # Left Foot
    lf = JointTrajectory()
    lf.joint_names = ['LAnklePitchJoint', 'LAnkleRoll']
    lfpt = JointTrajectoryPoint()
    lfpt.positions = [l_ankle_pitch_joint, l_ankle_roll]
    lfpt.time_from_start = rospy.Duration.from_sec(1)
    lf.points.append(lfpt)    
    
    # Right Foot
    rf = JointTrajectory()
    rf.joint_names = ['RAnklePitchJoint', 'RAnkleRoll']
    rfpt = JointTrajectoryPoint()
    rfpt.positions = [r_ankle_pitch_joint, r_ankle_roll]
    rfpt.time_from_start = rospy.Duration.from_sec(1)
    rf.points.append(rfpt)
        
    # Publishing all the joint position messages
    left_arm.publish(la)
    right_arm.publish(ra)
    left_hand.publish(lh)
    right_hand.publish(rh)
    pelvis.publish(p)
    left_leg.publish(ll)
    right_leg.publish(rl)
    left_foot.publish(lf)
    right_foot.publish(rf)
    
    print('Published joint trajectory test messages.')


def trajectory_gait():
    # Defining the joint trajectory publishers as global variables to use them inside of functions
    global left_arm, right_arm, left_hand, right_hand, pelvis, left_leg, right_leg, left_foot, right_foot
    
    # Initializing the ROS Node 
    rospy.init_node("JointPub")
    
    # Defining the publishers for each of the controllers
    left_arm = rospy.Publisher('/nao_dcm/LeftArm_controller/command', JointTrajectory, queue_size=1)
    right_arm = rospy.Publisher('/nao_dcm/RightArm_controller/command', JointTrajectory, queue_size=1)
    left_hand = rospy.Publisher('/nao_dcm/LeftHand_controller/command', JointTrajectory, queue_size=1)
    right_hand = rospy.Publisher('/nao_dcm/RightHand_controller/command', JointTrajectory, queue_size=1)    
    pelvis = rospy.Publisher('/nao_dcm/Pelvis_controller/command', JointTrajectory, queue_size=1)    
    left_leg = rospy.Publisher('/nao_dcm/LeftLeg_controller/command', JointTrajectory, queue_size=1)
    right_leg = rospy.Publisher('/nao_dcm/RightLeg_controller/command', JointTrajectory, queue_size=1)    
    left_foot = rospy.Publisher('/nao_dcm/LeftFoot_controller/command', JointTrajectory, queue_size=1)    
    right_foot = rospy.Publisher('/nao_dcm/RightFoot_controller/command', JointTrajectory, queue_size=1)    
    
    # Waiting for all publishers to establish connection with the controller listeners
    while left_arm.get_num_connections() < 1 or right_arm.get_num_connections() < 1 or left_hand.get_num_connections() < 1 or right_hand.get_num_connections() < 1 or pelvis.get_num_connections() < 1 or left_leg.get_num_connections() < 1 or right_leg.get_num_connections() < 1 or left_foot.get_num_connections() < 1 or right_foot.get_num_connections() < 1:
        pass
    
    # Calling the function that generates the messages and publishes
    pub_joint_states(0)
    
    rospy.Timer(rospy.Duration(0.01), pub_joint_states)
    rospy.spin()


if __name__ == '__main__':
    trajectory_gait()
