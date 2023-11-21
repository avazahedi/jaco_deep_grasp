# jaco_deep_grasp
Author: Ava Zahedi (Northwestern MSR 2023)  
In collaboration with NU Argallab and MSR.

# Setup - Dependencies and Docker
## Prerequisites
Import the necessary repositories listed in jaco_grasp.repos using vcs tool. To do so, clone this repository into the src directory of your workspace. Then, in the root of your workspace, run the following:  
`vcs import < src/jaco_deep_grasp/jaco_grasp.repos`

The following repositories should be added to your workspace in their desired branches:
- deep_grasp_demo (realsense)
- moveit_task_constructor (realsense)
- jaco_base (msr23-grasp)

This repository (jaco_deep_grasp) should be in the nodes branch.

Make sure the kinova-ros repository inside jaco_base is correctly cloned.  
```
cd src/jaco_base
git submodule update --init --recursive
cd kinova-ros
git switch msr23-grasp
```

## Set up Docker
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
where jaco_deep_grasp_cntr is the container name.

Subsequent steps should be done in Docker.  

## Build the workspace
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

## Camera
If collecting point cloud data from a camera, make sure to update the transform from root to the camera in `jaco_deep_grasp.launch`.  
Also update the `trans_base_cam` transform in `camera.yaml`.

## Place pose
Set the desired place pose after grasping by modifying `place_pose` in `kinova_object.yaml`.

## Run deep grasp demo in simulation (Rviz)
Source the workspace
```
source devel/setup.bash
```

Run the jaco_deep_grasp launch file
```
roslaunch jaco_grasp_ros jaco_deep_grasp.launch
```

## Run deep grasp demo on hardware
Launch jaco_base
```
roslaunch jaco_interaction jaco_base.launch
```

In another terminal, home the arm and set the control mode to trajectory
```
rosservice call /j2s7s300_driver/in/home_arm
rosservice call /j2s7s300_driver/in/set_control_mode "current_control_mode: trajectory"
```

Launch jaco_deep_grasp
```
roslaunch jaco_grasp_ros jaco_deep_grasp.launch fake_execution:=false
```

