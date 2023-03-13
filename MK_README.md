# PX4 Firmware
## Checks
Check any updates before cloning.

```
sudo apt-get update -y
sudo apt-get upgrade -y
sudo apt install git -y
```
## Depends Before Cloning
These installations solved all of the errors I faced.
```
sudo apt install python3-pip -y
pip3 install kconfiglib
sudo apt install gcc-arm-none-eabi -y
pip3 install --user jinja2
pip3 install --user jsonschema
```

## Cloning PX4
Clone the branch `MK` from `Mu99-M/PX4-Autopilot` in home directory.
```
cd ~
git clone -b MK https://github.com/Mu99-M/PX4-Autopilot.git --recursive
```
## Install gstreamer for SITL
```
sudo apt-get install libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev libgstreamer-plugins-bad1.0-dev gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly gstreamer1.0-libav gstreamer1.0-doc gstreamer1.0-tools gstreamer1.0-x gstreamer1.0-alsa gstreamer1.0-gl gstreamer1.0-gtk3 gstreamer1.0-qt5 gstreamer1.0-pulseaudio -y
```

# Install ROS Noetic

# Install Mavros

# Install QGroundControl (QGC)

