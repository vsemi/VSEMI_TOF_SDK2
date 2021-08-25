
mkdir build

cd build

cmake ..

make

sudo chmod a+rw /dev/ttyACM0

./camera_info_reader


