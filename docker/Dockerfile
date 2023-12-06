ARG image=nvidia/cudagl:11.3.0-devel-ubuntu20.04
FROM ${image}

ARG DEBIAN_FRONTEND=noninteractive
# ARG workstation_ip=192.168.0.1
# ARG use_robotiq=0
# ARG build_dope=1
# ARG build_contactgraspnet=1
# ARG build_megapose=1
ENV WS_DIR="/root/git"
WORKDIR ${WS_DIR}

SHELL ["/bin/bash", "-c"]
# Dependencies for latency plot
RUN apt-get update && apt-get upgrade -y && apt-get install -y \
    curl \
    grep \
    build-essential \
    make \
    libnuma-dev \
    python3 \
    python3-distutils \
    gnuplot-qt \ 
    sshpass \
    iproute2 \
    python3-pip \
    gnome-terminal\
    nano\
    vim \
    git
# Dependencies for Bash unit-tests
RUN apt-get update && apt-get install -y \
    bats \
    dialog \
    tmux \
    iputils-ping 	

# ROS Install
RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
RUN echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" > /etc/apt/sources.list.d/ros-latest.list

RUN apt-get update && apt-get install --no-install-recommends -y \
    python3-rosdep \
    python3-rosinstall \
    python3-vcstools \
    python3-catkin-tools \
    ros-noetic-jsk-tools \
    && rm -rf /var/lib/apt/lists/*

RUN rosdep init && \
    rosdep update

RUN apt-get update && apt-get install -y \
    ros-noetic-desktop-full \
    git \
    && rm -rf /var/lib/apt/lists/*

RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
RUN echo "source /root/git/jango_ws/devel/setup.bash" >> ~/.bashrc

ENV ROS_DISTRO noetic


RUN echo "copying jango ros packages"
RUN mkdir /root/jango_ws/src -p \
    && source /opt/ros/noetic/setup.bash\ 
    && apt-get update && apt-get -y upgrade 

COPY ./src /root/jango_ws/src
RUN apt-get install -y ros-noetic-ridgeback-desktop ros-noetic-catkin   ros-noetic-serial libserial-dev\ 
    &&  python3 -m pip install conan==1.59 \
    && conan config set general.revisions_enabled=1

RUN cd /root/jango_ws && rosdep install --from-paths src --ignore-src -r -y  \
    && source /opt/ros/noetic/setup.bash\ 
    && catkin_make 
# && catkin_init_workspace \
# && git clone https://github.com/lardemua/atom/tree/noetic-devel \
# && https://github.com/miguelriemoliveira/mmtbot \
# && git clone https://github.com/iris-ua/iris_ur10e -b noetic-devel \
# && git clone https://github.com/iris-ua/iris_ur10e_calibration -b noetic-devel\
# && git clone https://bitbucket.org/DataspeedInc/velodyne_simulator -b master\
# && git clone -b calibration_devel https://github.com/lardemua/universal_robot.git \
# && rm -r iris_ur10e/ur_kinematics 

# RUN cd /root/django_ws && catkin_make_isolated 

SHELL ["conda", "run", "-n", "myenv", "/bin/bash", "-c"]


# && rosdep install --from-paths src -i --rosdistro ${ROS_DISTRO} \