# jaco_deep_grasp

## Prerequisites
Import the necessary repositories listed in jaco_grasp.repos using vcs tool. To do so, clone this repository into the src directory of your workspace. Then, in the root of your workspace, run the following:  
`vcs import < src/jaco_deep_grasp/jaco_grasp.repos`

The following repositories should be added to your workspace in their desired branches:
- deep_grasp_demo (jaco_demo)
- moveit_task_constructor (jaco_demo)
- jaco_base (msr23-grasp)
- gpd (master)

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
## ~~Set up GPD~~ Skip this for now
Currently this step is done in creating the Docker image, so no need to do it manually.
```
cd gpd
mkdir build
cd build
cmake ..
make -j
make install
```

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


## Run deep grasp demos
Source the workspace
```
source devel/setup.bash
```

Run the jaco_deep_grasp launch file
```
roslaunch jaco_grasp_ros jaco_deep_grasp.launch
```