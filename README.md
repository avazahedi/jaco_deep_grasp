# jaco_deep_grasp
Author: Ava Zahedi (Northwestern MSR 2023)  
In collaboration with NU Argallab and MSR.

# Overview
This project builds on [MoveIt Deep Grasps](https://ros-planning.github.io/moveit_tutorials/doc/moveit_deep_grasps/moveit_deep_grasps_tutorial.html) to implement autonomous grasping capabilities on the 7-DOF Kinova Jaco Gen2 arm (model j2s7s300).  

The pipeline begins with a RealSense camera that publishes the following to ROS topics: point cloud data, RGB image, and aligned depth image. The RGB and depth images are used with YOLOv8 for object detection, labeling, and segmentation. This information is then used to perform segmentation and filtering on the point cloud.  
The segmented cloud is passed into Grasp Pose Detection (GPD), a neural network used to generate viable grasp poses given a point cloud of the desired object to grasp. From these grasps, a motion plan is constructed to be used with MoveIt so that the arm can navigate to, pick, and place the object.  

Originally designed in simulation only and on a Franka Panda robot, some of my modifications and additions from MoveIt Deep Grasps include:
* Adaptation to the Kinova Jaco arm
* Translation to live hardware trajectory execution
* Live point cloud generation, segmentation, and filtering
* YOLOv8 object detection to provide semantic labeling and segmentation parameters for the object to grasp

# Setup - Dependencies and Docker
## Prerequisites
Import the necessary repositories listed in jaco_grasp.repos using vcs tool. To do so, clone this repository into the src directory of your workspace. Then, in the root of your workspace, run the following:  
`vcs import < src/jaco_deep_grasp/jaco_grasp.repos`

The following repositories should be added to your workspace in their desired branches:
- deep_grasp_demo (realsense)
- moveit_task_constructor (realsense)
- jaco_base (msr23-grasp)

Make sure the kinova-ros repository inside jaco_base is correctly cloned.  
```
cd src/jaco_base
git submodule update --init --recursive
cd kinova-ros
git switch msr23-grasp
```

The `main` and `nodes` branches of this repository (jaco_deep_grasp) should be the same, and either will work for this demo.

## Hardware Requirements
* Intel RealSense Depth Camera D435i (both simulation and hardware)
    * Use a USB 3.2 or better cable for best results
* 7-DOF Kinova Jaco Gen2 arm - j2s7s300 (only required for hardware demo)

## Set Up Docker
First, build the image from the included Dockerfile  
```
cd src/jaco_deep_grasp
sudo docker build -t jaco_deep_grasp_image .
```
where `jaco_deep_grasp_image` is the name of the image.

Then, build the container with the necesssary mounts:
```
sudo docker run -it --privileged \
-v /dev/bus/usb:/dev/bus/usb \
-v /dev/input/by-id:/dev/input/by-id \
-v /tmp/.X11-unix:/tmp/.X11-unix:rw \
-v /home/user/jaco_grasp_ws:/home/user/jaco_grasp_ws \
-e DISPLAY \
-e QT_X11_NO_MITSHM=1 \
--name jaco_deep_grasp_cntr \
--net=host \
jaco_deep_grasp_image
```
where `jaco_deep_grasp_cntr` is the container name.

Subsequent steps should be done in Docker.  

## Build the Workspace
```
# cd into jaco_grasp_ws
catkin build
```

Note: There may be many warnings with the kinova packages. These can be ignored for now.

If there are missing dependencies, try the following and then build again.
```
apt-get update
rosdep install -y --from-paths . --ignore-src --rosdistro noetic
```

If there is an issue getting python-pyudev, do  
```
apt-get install -y python3-pyudev
```

# Setup - Demo
## jaco_deep_grasp launch file
jaco_deep_grasp.launch launchfile arguments
* `load_cloud` defaults to false
    * True if loading a pcd file directly. Set to false if listening to a ROS topic to receive point cloud data.
    * If true, modify the `path_to_pcd_file` arg in `gpd_demo.launch` to the correct file path.
    * If false, modify the `point_cloud_topic` arg in `gpd_demo.launch` to the correct topic.
* `fake_execution` defaults to true
    * True if just running the demo in Rviz. Set to false if connecting to the hardware and performing real execution.
* `desired_object` defaults to bottle
    * The class of the object you want to pick up. Some good options with the currently loaded YOLO model are cup and bottle.

## Camera
If collecting point cloud data from a RealSense camera, make sure to update the transform from root to the camera in `jaco_deep_grasp.launch`.  
Also update the `trans_base_cam` transform in `camera.yaml`.

## Properties of Object to be Picked
This is for creating a collision object to represent the object being picked. This gets used in generating the motion plan.  
Set the `object_pose` and `object_dimensions` in `kinova_object.yaml`.  
* See the **Unfinished Work** section at the end of the README for information regarding my progress towards dynamic collision object generation.

## Place Pose
Set the desired place pose after grasping by modifying `place_pose` in `kinova_object.yaml`.

# Running Jaco Deep Grasp
## Run Deep Grasp Demo in Simulation (Rviz)
Source the workspace
```
source devel/setup.bash
```

Run the jaco_deep_grasp launch file
```
roslaunch jaco_grasp_ros jaco_deep_grasp.launch
```

## Run Deep Grasp Demo on Hardware
Launch jaco_base
```
roslaunch jaco_interaction jaco_base.launch
```

In another terminal, home the arm and set the control mode to trajectory
```
rosservice call /j2s7s300_driver/in/home_arm
rosservice call /j2s7s300_driver/in/set_control_mode "current_control_mode: trajectory"
```

In another terminal, launch jaco_deep_grasp
```
roslaunch jaco_grasp_ros jaco_deep_grasp.launch fake_execution:=false
```

# Live Demo
This is the live video along with the corresponding computer screencast showing YOLO object detection and visualization in Rviz.  

https://github.com/avazahedi/jaco_deep_grasp/assets/39091881/68a3b8e9-68c6-458e-a7c5-8815961fa89a

https://github.com/avazahedi/jaco_deep_grasp/assets/39091881/3b010bf4-64fc-47da-9618-b5e05e032a34

# Notes
### Error Fix: Kinematics plugin (arm) failed to load
When running the jaco_deep_grasp.launch launch file, if there is an error: `The kinematics plugin (arm) failed to load.` do the following inside your Docker container:
```
apt purge -y ros-noetic-moveit*
apt update
apt install -y ros-noetic-moveit*
apt install -y ros-noetic-geometric-shapes
apt install -y ros-noetic-srdfdom
apt install -y ros-noetic-ompl
apt install -y ros-noetic-trac-ik-kinematics-plugin
```

Then, rebuild your workspace with `catkin build`

This happens because of conflicting versions of MoveIt packages in the container. Uninstalling and reinstalling MoveIt makes sure the versions are consistent.

### Plotting Trajectory Execution on Hardware
One challenge I had with this project was having a good plan generated in MoveIt but inconsistent trajectory execution on the hardware. The `traj_plot.py` node can be run separately when launching the demo with `fake_execution:=false`. This will generate graphs following the trajectory execution that show the desired and actual positions of each of the joints, as well as the error between them. This node is purely informative and meant to give a better understanding of whether or not the hardware is properly calibrated. 

### Unfinished Work - Dynamic Collision Object Generation
The `generate-collision-object` branch in the deep_grasp_demo and jaco_deep_grasp repositories contain unfinished work towards dynamically generating a collision object based on information from YOLO. This would enable the user to place the object anywhere in the workspace and have MoveIt plan to an object location without having to predefine it in the kinova_object.yaml file that gets loaded into the ROS parameter server. Functionality for placing the object anywhere is already in place for point cloud segmentation and YOLO object detection.  
The main issue I was running into was figuring out how to send this newly-generated object as a goal to MoveIt using the existing architecture of deep grasp.  
