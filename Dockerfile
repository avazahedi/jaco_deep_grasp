FROM argallab/focoetic:latest

# Install packages without prompting the user to answer any questions
ENV DEBIAN_FRONTEND noninteractive 
ENV ROS_DISTRO noetic

# Dependencies for glvnd and X11.
RUN apt-get update \
  && apt-get install -y -qq --no-install-recommends \
    libglvnd0 \
    libgl1 \
    libglx0 \
    libegl1 \
    libxext6 \
    libx11-6 \
  && rm -rf /var/lib/apt/lists/*

# Install packages
RUN apt-get update && apt-get install -y \
wget \
git \
lsb-release \
mesa-utils \
byobu \
build-essential \
dbus-x11 \
dirmngr \
gnupg2 \
libx11-dev \
libxau-dev \
libxcb1-dev \
libxdmcp-dev \
libxext-dev \
pkg-config \
ssh \
udev \
swig \
vim 
RUN apt-get clean && rm -rf /var/lib/apt/lists/*

# RUN mkdir ~/.ssh && ln -s /run/secrets/host_ssh_key ~/.ssh/id_rsa

# make sure your domain is accepted
# RUN touch /root/.ssh/known_hosts
# RUN ssh-keyscan github.org >> /root/.ssh/known_hosts

# RUN apt-get update -y && apt-get install -y \
#     ros-$ROS_DISTRO-desktop-full \
#     && rm -rf /var/lib/apt/lists/*

# Install new paramiko (solves ssh issues)
# RUN apt-add-repository universe
RUN apt-get update && apt-get install -y python3-pip python3 build-essential && apt-get clean && rm -rf /var/lib/apt/lists/*
RUN /usr/bin/yes | pip3 install --upgrade pip

RUN apt-get update -y && apt-get install -y \    
    python3-scipy \
    python3-numpy \ 
    python3-cvxopt \
    python3-pandas \ 
    ros-$ROS_DISTRO-joy \
    ros-$ROS_DISTRO-rosserial-arduino \
    ros-$ROS_DISTRO-moveit \
    && apt-get clean && rm -rf /var/lib/apt/lists/* 

RUN pip3 install matplotlib pymdptoolbox box2d-py tk pyglet pyudev ipython


####DEEPGRASP####
## PCL
RUN apt install libpcl-dev

## OpenCV
# dependencies
RUN apt install build-essential \
    cmake \
    git \
    libgtk2.0-dev \
    pkg-config \
    libavcodec-dev \
    libavformat-dev \
    libswscale-dev \
    python3-dev \
    # python3-numpy \
    libtbb2 \
    libtbb-dev \
    libjpeg-dev \
    libpng-dev \
    libtiff-dev \
    # libjasper-dev \
    libdc1394-22-dev \
    libavcodec-dev \
    libavformat-dev \
    libswscale-dev
    # libv4l-dev \
    # libxvidcore-dev \
    # libx264-dev \
    # libgtk-3-dev \
    # libatlas-base-dev \
    # gfortran pylint

RUN apt install python3-opencv

## GPD library
RUN git clone https://github.com/avazahedi/gpd \
    && cd gpd \
    && mkdir build && cd build \
    && cmake .. \
    && make -j \
    && make install

## Dex-Net
# installing Tensorflow 2.12 instead of 1.15
# python3-vtk7 instead of python-vtk6
# libvtk7-dev instead libvtk6-dev
# skipping python-qt4 (looks like python3-pyqt5 is already installed)

# DON'T DO dexnet_requirements.txt - messes up the version compatabilities
# doing them manually
# dill is installed with multiprocess
RUN python3 -m pip install --upgrade pip \
    && pip install tensorflow \
    && apt update \
    && apt install -y cmake libvtk7-dev python3-vtk7 python3-sip libosmesa6-dev meshlab libhdf5-dev \
    # dexnet_requirements.txt here to end of run statement
    && pip install scikit-learn \
    && pip install scikit-image \ 
    && pip install multiprocess \
    # && pip install dill \
    && pip install cvxopt==1.2.5 \
    && pip install trimesh

# install deps from source
    # install autolab modules
# RUN cd /home \
#     && mkdir dexnet_deps \
#     && cd dexnet_deps \
#     && git clone https://github.com/BerkeleyAutomation/autolab_core.git \
#     && git clone https://github.com/BerkeleyAutomation/perception.git \
#     && git clone https://github.com/BerkeleyAutomation/gqcnn.git \
#     && git clone https://github.com/BerkeleyAutomation/visualization.git \
#     && cd autolab_core \
#     && python3 setup.py develop \
#     && cd .. \
#     && cd perception \
#     && python3 setup.py develop \
#     && cd .. \
#     && cd gqcnn \
#     && python3 setup.py develop \
#     && cd .. \
#     && cd visualization \
#     && python3 setup.py develop \
#     && cd .. 

RUN apt install -y python3-catkin-tools python3-osrf-pycommon

####DEEPGRASP####

# for jaco_base
RUN apt install -y ros-noetic-trac-ik-kinematics-plugin

# # by default, open in /home/user directory
# RUN cd /home/avaz
# # add re-source alias
# RUN echo "alias rs='source ~/.bashrc'" >> ~/.bashrc

# X windows access
ENV DISPLAY=$DISPLAY
ENV QT_X11_NO_MITSHM=1

# setup entrypoint
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc

# Entry script
COPY ./ros_entrypoint.sh /
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]