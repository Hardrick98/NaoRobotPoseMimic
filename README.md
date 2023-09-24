# NaoRobotPoseMimic

Ensure you have a working webcam on your device.

## 1. Activate the Gazebo/MoveIt simulation

```
roslaunch nao_moveit_config demo_gazebo.launch
```
Runs the simulation and starts moveit.

## 2. Estimate the 3D Pose with the Webcam

```
roslaunch pose_estimation pose.launch
```
pose_estimation is a package created thanks to the splendid work of https://github.com/facebookresearch/VideoPose3D.
This package estimate the 3D reconstruction of your body and sends the information into an appropriate topic.

## 3. Choose which arm to move (left or right)

```
roslaunch nao_trajectory right.py
```
Retrieves the information from the specific topic and computes the trajectory with IK solver to reach the desired hand position (ignoring orientation)

