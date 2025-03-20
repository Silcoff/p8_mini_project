FROM ubuntu:jammy

# Avoid prompts with apt
ARG DEBIAN_FRONTEND=noninteractive

# Set Python3 as default
RUN apt update && apt install -y python3
RUN update-alternatives --install /usr/bin/python python /usr/bin/python3 1

###################################
###         Install ROS         ###
###################################

# Update system locale
RUN apt install -y locales && \
    locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

ENV LANG=en_US.UTF-8
ENV PYTHONIOENCODING=utf-8


# 
RUN apt install software-properties-common -y && \
    add-apt-repository universe
    
# Add the ROS 2 GPG key with apt.
RUN apt update && apt install curl -y && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# Add the repository to your sources list.
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" > /etc/apt/sources.list.d/ros2.list

RUN apt update && apt upgrade -y && \
    apt install ros-humble-desktop -y

RUN echo "source /opt/ros/humble/setup.bash" >> $HOME/.bashrc
