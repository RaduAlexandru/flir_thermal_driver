driver for FLIR Boson 640 thermal camera.

dependencies
sudo apt-get install libjpeg-dev libusb-1.0-0-dev

installation: 
catkin build --this

sudo cp 99-flir.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && udevadm trigger

add current user to group:
sudo useradd -g <userName> usb

run the boson or the lepton camera:
roslaunch flir_thermal_driver run_flir_boson.launch
roslaunch flir_thermal_driver run_flir_lepton.launch

If you get error: uvc_open: Access denied (-3)
    check that you have permission for /dev/videoX and /dev/bus/usb/
    sudo chown -R <userName> /dev/bus/usb/001
