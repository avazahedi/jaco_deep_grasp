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
# Install DeepGrasp stuff
# RUN wget https://raw.githubusercontent.com/PickNikRobotics/deep_grasp_demo/master/pcl_install.sh && chmod +x pcl_install.sh && sudo ./pcl_install.sh
# RUN wget https://raw.githubusercontent.com/PickNikRobotics/deep_grasp_demo/master/opencv_install.sh && chmod +x opencv_install.sh && sudo ./opencv_install.sh

## PCL
# RUN wget https://github.com/PointCloudLibrary/pcl/archive/pcl-1.11.0.zip -O pcl-pcl-1.11.0.zip
#     # && apt install unzip \
#     # && unzip pcl-pcl-1.11.0.zip \
#     # && cd pcl-pcl-1.11.0 \
#     # && mkdir build && cd build \
#     # && cmake -DCMAKE_BUILD_TYPE=Release .. \
#     # && make -j4 \
#     # && make install

# print working directory
# RUN pwd && ls

# RUN mkdir /home/new_dockerfile 
    # && cd /home/new_dockerfile

# WORKDIR /home/avaz/new_dockerfile

# ADD pcl-pcl-1.11.0.zip /home/new_dockerfile

# RUN apt install unzip \
#     && cd /home/new_dockerfile \
#     && unzip pcl-pcl-1.11.0.zip \
#     && cd pcl-pcl-1.11.0 \
#     && mkdir build && cd build \
#     && cmake -DCMAKE_BUILD_TYPE=Release .. \
#     && make -j4 \
#     && make install
RUN apt install libpcl-dev
## PCL


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

# Download and build OpenCV
# RUN wget https://github.com/opencv/opencv/archive/3.4.0.zip -O opencv-3.4.0.zip \
#     && wget https://github.com/opencv/opencv_contrib/archive/3.4.0.zip -O opencv_contrib-3.4.0.zip \
#     && apt install unzip \
#     && unzip opencv-3.4.0.zip \
#     && unzip opencv_contrib-3.4.0.zip \
#     && cd  opencv-3.4.0 \
#     && mkdir build \
#     && cd build \
#     && cmake -DCMAKE_BUILD_TYPE=Release -D WITH_CUDA=OFF -DCMAKE_INSTALL_PREFIX=/usr/local -DOPENCV_EXTRA_MODULES_PATH=../../opencv_contrib-3.4.0/modules -DOPENCV_ENABLE_NONFREE=True .. \
#     && make -j4 \
#     && make install \
#     && ldconfig
## OpenCv

## GPD library
# RUN cd /home/avaz/new_dockerfile/gpd \
#     && mkdir build \
#     && cd build \
#     && cmake .. \
#     && make -j \
#     && make install     # builds to 100% but see error in Final Project Resources google doc - this is issue with unmodified version
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
RUN cd /home \
    && mkdir dexnet_deps \
    && cd dexnet_deps \
    && git clone https://github.com/BerkeleyAutomation/autolab_core.git \
    && git clone https://github.com/BerkeleyAutomation/perception.git \
    && git clone https://github.com/BerkeleyAutomation/gqcnn.git \
    && git clone https://github.com/BerkeleyAutomation/visualization.git \
    && cd autolab_core \
    && python3 setup.py develop \
    && cd .. \
    && cd perception \
    && python3 setup.py develop \
    && cd .. \
    && cd gqcnn \
    && python3 setup.py develop \
    && cd .. \
    && cd visualization \
    && python3 setup.py develop \
    && cd .. 
    
# RUN cd gqcnn \
#     # && wget https://drive.google.com/uc?id=1fbC0sGtVEUmAy7WPT_J-50IuIInMR9oO&export=download
#     && wget --load-cookies /tmp/cookies.txt "https://docs.google.com/uc?export=download&confirm=$(wget --quiet --save-cookies /tmp/cookies.txt --keep-session-cookies --no-check-certificate 'https://docs.google.com/uc?export=download&id=FILEID' -O- | sed -rn 's/.*confirm=([0-9A-Za-z_]+).*/\1\n/p')&id=FILEID" -O FILENAME && rm -rf /tmp/cookies.txt

######## can't cd because haven't mounted yet - do this after creating the image using the dexnet_setup.sh file
# once the models are in
# RUN && cd gqcnn/models \
#     && unzip -a GQCNN-2.0.zip \
#     && mv GQ-Image-Wise GQCNN-2.0 \
#     && unzip -a GQCNN-2.1.zip \
#     && mv GQ-Bin-Picking-Eps90 GQCNN-2.1 \
#     && unzip -a GQCNN-3.0.zip \
#     && mv GQ-Suction GQCNN-3.0 \
#     # missing these two 
#     # && unzip -a GQCNN-4.0-PJ.zip \
#     # && unzip -a GQCNN-4.0-SUCTION.zip \
#     && unzip -a FC-GQCNN-4.0-PJ.zip \
#     && unzip -a FC-GQCNN-4.0-SUCTION.zip \
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