The sample 3, to work with OpenCV and PCL to show depth map and point cloud obtained from ToF sensor, which requires ROS Melodic or PCL as dependencies.

Prerequisites:

  x64
  ubuntu
  OpenCV
  PCL

Build and run the sample application

1. Plug in the sensor, and set USB permission:

  >> sudo chmod a+rw /dev/ttyACM0

2. Build sample application:

  >> mkdir build && cd build

  >> cmake .. -DTARGET_PLATFORM=x64_ubuntu

  >> make

3. To start the sample application, run command:

  >> ./vcam

4. To stop the application, press Esc key when the depth map window active, or use mouse to close the point cloud window.


