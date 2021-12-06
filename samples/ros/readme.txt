Prerequisites:
  ROS

Install ROS Noetic

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


6. Settings:

  If the FoV of your sensor is not default FoV (55x21), makre sure change the value of angle_x and angle_y, from the configuration panel:

![Config](screenshot_config.png)

  Or set FoV in source code by calling a method in Camera instance: setFoV(double angle_x, double angle_y)
  
