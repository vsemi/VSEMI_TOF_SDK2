Prerequisites:
=============

  Ubuntu
  ROS Noetic

Install ROS Noetic
==================

  >> sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

  >> sudo apt install curl # if you haven't already installed curl
     curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

  >> sudo apt update
  
  >> sudo apt install ros-noetic-desktop-full

  >> echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
  
  >> source ~/.bashrc

  >> sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
    
  >> sudo rosdep init
  
  >> rosdep update
  
  ## set env for root so that application can be ran using sudo command

  >> sudo -s
  
  >> echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
  
  >> exit

Build sample application:
=======================

  ## make sure the file src/vsemi_tof_ros/cfg/vsemi_tof_ros.cfg has execute permission:

  >> sudo chmod a+rxw src/vsemi_tof_ros/cfg/vsemi_tof_ros.cfg

  >> catkin_make

Run sample application under root:
=================================

1. Plug in the sensor, and set USB permission:

2. To start the ROS sample application, run command:

  ## If it is first time to run ROS sample, please make sure "run.sh" has execute permission:

  >> sudo chmod a+rxw run.sh

  >> sudo -s

  >> ./run.sh
  
3. Turn on check box "Point Cloud 0 - 5" (refer to screenshot) to view point cloud captured by each sensor.

4. To stop ROS, please press Ctr + C in terminal 


