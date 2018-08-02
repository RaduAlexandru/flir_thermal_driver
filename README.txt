driver for FLIR Boson 640 thermal camera.

installation: 
cd libuvc
mkdir build && cd build
cmake .. 
make
cd ..
catkin build --this

copy 40-pgr.rules to /etc/udev/rules.d/
udevadm control --reload-rules && udevadm trigger

add current user to group:
useradd -G flirimaging <userName>

run:
roslaunch run_flir_boson.launch
