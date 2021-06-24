Working with ROS to show depth map and point cloud obtained from ToF sensor, which requires ROS as dependency.

Prerequisites:
  ROS

Install ROS Melodic

  >> sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

  >> sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

  >> sudo apt update

  >> sudo apt install ros-melodic-desktop-full

  >> sudo rosdep init

  >> rosdep update

  >> echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc

  >> source ~/.bashrc

  >> sudo apt install python-rosinstall python-rosinstall-generator python-wstool build-essential
  
  ## If there is no error throughout the above installation process, dependencies should have been installed successfully.

Build and run sample application:

1. Plug in the sensor, and set USB permission:

  >> sudo chmod a+rw /dev/ttyACM0

3. Build sample ROS application, 

  ## make sure the file src/vsemi_tof_ros/cfg/vsemi_tof_ros.cfg has execute permission:

  >> sudo chmod a+rxw src/vsemi_tof_ros/cfg/vsemi_tof_ros.cfg

  >> catkin_make

4. To start the ROS sample application, run command:

  ## If it is first time to run ROS sample, please make sure "run.sh" has execute permission:

  >> sudo chmod a+rxw run.sh

  ## Then run command to start ROS sample application:

  >> ./run.sh

5. To stop ROS, please press Ctr + C in terminal 

