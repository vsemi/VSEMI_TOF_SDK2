The sample 3, to work with OpenCV and PCL to show depth map and point cloud obtained from ToF sensor.

Prerequisites:
  OpenCV
  PCL

Build and run the sample application

1. Plug in the sensor, and set USB permission:

  >> sudo chmod a+rw /dev/ttyACM0

2. Build sample application:

  >> cd build

  >> cmake ..

  >> make

3. To start the sample application, run command:

  >> ./camera

4. To stop the application, press Esc key when the depth map window active, or press 'q' key or use mouse to close the point cloud window.

