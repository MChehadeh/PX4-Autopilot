### These instructions are used for Ubuntu 20.04. They could be used in WSL2 or as a dual boot. But it's highly recommended to be used as a dual boot.

# Overview
- This is a re-published (not forked) repo of the original PX4-Autopilot. It also includes the settings file of PX4 for different variants of UAVs, and the relevant setup information. The main applications located in the `src/modules` directory.

- This repo is mainly used for:
    1. Testing SITL with offboard mode.
    2. Testing on hardware (pixhawk).

## Table of Contents

1. [Setup of Companion Computer / PC](#Setup_of_Companion_Computer)
    1. [PX4-Autopilot Installation](#PX4-Autopilot_Installation)
    2. [Mavros and Offboard Installation](#Mavros_and_Offboard_Installation)
    3. [QGroundControl](#QGroundControl)
2. [Run SITL in offboard mode](#How_to_run_SITL_in_offboard_mode)
3. [Setup of Pixhawk](#Setup_of_Pixhawk)
    1. [Loading pre-built firmware](#pre-built)
    2. [Building custom firmware](#custom)

# Setup of Companion Computer / PC <a name="Setup_of_Companion_Computer"></a>
We will need to install PX4-Autopilot, Mavlink, Mavros to be able to have a communication between PX4 and ROS.

## PX4-Autopilot Installation <a name="PX4-Autopilot_Installation"></a>
- Install dependencies:

```bash
sudo apt install python3-pip -y
pip3 install kconfiglib
sudo apt install gcc-arm-none-eabi -y
pip3 install --user jinja2
pip3 install --user jsonschema
sudo apt-get install libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev libgstreamer-plugins-bad1.0-dev gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly gstreamer1.0-libav gstreamer1.0-doc gstreamer1.0-tools gstreamer1.0-x gstreamer1.0-alsa gstreamer1.0-gl gstreamer1.0-gtk3 gstreamer1.0-qt5 gstreamer1.0-pulseaudio -y
```

- Clone `PX4-Autopilot` into home directory:

```bash
cd ~
git clone https://github.com/Mu99-M/PX4-Autopilot.git --recursive
```

## Mavros and Offboard Installation <a name="Mavros_and_Offboard_Installation"></a>
- The offb node has 3 jobs:
    1. Switch to offboard mode and checks if it's disconnected to try switching again.
    2. Arm the vehicle and checks if it's disarmed to try arming again.
    3. Publishing values to SITL.

- Install dependencies:

```bash
sudo apt install python3-catkin-tools python3-rosinstall-generator python3-osrf-pycommon -y
pip3 install future
```

- Create a workspace:

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone https://github.com/Mu99-M/offboard_testing.git --recursive
cd ..
catkin build
source devel/setup.bash
```

Now everthing is ready to be used from companion computer side.

## QGroundControl QGC (Optional) <a name="QGroundControl"></a>
- QGroundControl is a open-source ground control station software that let us to plan, monitor, and control vehicles. With features ranging from mission planning and telemetry monitoring to vehicle configuration and real-time control.

- QGroundControl can be installed on Ubuntu LTS 20.04 (and later). Please follow the instructions in [this link](https://docs.qgroundcontrol.com/master/en/getting_started/download_and_install.html#ubuntu) for QGC installation.

# How to run SITL in offboard mode <a name="How_to_run_SITL_in_offboard_mode"></a>
1. Build px4 in sitl

```bash
cd ~/PX4-Autopilot/
make px4_sitl_default none
```

2. Launch px4.launch node

```bash
roslaunch mavros px4.launch fcu_url:=udp://:14550@14557
```

3. Launch the starting node

```bash
roslaunch offb starting.launch
```

# Setup of Pixhawk / PX4 <a name="Setup_of_Pixhawk"></a>
You can either use [(1) pre-buit](#pre-built) firmware files or [(2) custom build](#custom) the firmware from source. Both procedures are describe below:

## 1. Loading pre-built firmware to Pixhawk: <a name="pre-built"></a>

You can find the files for pre-built firmware in the `PX4-Autopilot/compiled firmware` directory. Upload the firmware suitabe for your FMU version to your pixhawk using QGroundControl as explained in this [guide](https://docs.qgroundcontrol.com/master/en/SetupView/Firmware.html).


## 2. Building custom firmware from source: <a name="custom"></a>
### Building PX4:
To build for Pixhawk 4 (FMUv5):
```bash
cd ~/PX4-Autopilot
make px4_fmu-v5_default
```
For any other version check [this link](https://docs.px4.io/main/en/dev_setup/building_px4.html#building-for-nuttx).

### Uploading firmware:
Upload the firmware to your pixhawk using QGroundControl as explained in this [guide](https://docs.qgroundcontrol.com/master/en/SetupView/Firmware.html).

# Setup ethernet communication with pixhawk (Optional):
You can refer to this [guide](https://docs.px4.io/main/en/advanced_config/ethernet_setup.html) to setup ethernet communication with you pixhawk if your pixhawk supports this (e.g. pixhawk 5x or 6x). If using the pixhawk cm4 baseboard with raspberry pi, you can refer to this [guide](https://docs.holybro.com/autopilot/pixhawk-baseboards/pixhawk-rpi-cm4-baseboard/ethernet-connection). The process to setup ethernet comunication with pixhawk is briefly explained below in the report in `1. Using Ethernet` subsection.


## PX4 parameter configurations
To configure the PX4 parameters correctly, you can directly upload a pre-configures file to your pixhawk / SITL. You can also configure the needed parameters manually as describe later in this guide.

### Loading pre-configured parameters set
You can find pre-configured parameters files on [this link](https://drive.google.com/drive/folders/1zgydF3k8lJNwXT9q2XfnUk0Gt9lqb0XE?usp=drive_link). You can upload the pre-configured parameter file to pixhawk using QGroundControl as described in [this link](https://docs.qgroundcontrol.com/master/en/SetupView/Parameters.html)

### Manually configure paramaters (Optional)
You can change px4 parameters from QGroundControl as described [here](https://docs.qgroundcontrol.com/master/en/SetupView/Parameters.html) The major needed parameter changes are described below:


#### Configuring the MAVLINK instance

##### 1. Using Ethernet

If you have already configured a static IP for your pixhawk and companion computer as explained in this [guide](https://docs.px4.io/main/en/advanced_config/ethernet_setup.html), you can skip to step 8.
1. Connect the pixhawk to QGC using a USB-C cable.
2. Open QGroundcontrol > Analyze Tools > MAVLink Console.
3. Enter the following commands: \
   echo DEVICE=eth0 > /fs/microsd/net.cfg \
                                        echo BOOTPROTO=static >> /fs/microsd/net.cfg\
                                        echo IPADDR=192.168.0.4 >> /fs/microsd/net.cfg\
                                        echo NETMASK=255.255.255.0 >>/fs/microsd/net.cfg\
                                        echo ROUTER=192.168.0.1 >>/fs/microsd/net.cfg\
                                        echo DNS=192.168.0.1 >>/fs/microsd/net.cfg\
   The IPADDR could be chosen by the user. Our default configuration use `IPADDR=192.168.0.4` as the IP address of the pixhawk. The pixhawk subnet should match the subnet of the companion computer.
4. Connect the pixhawk to an external power source, and connect the ethernet cable to the companion computer / PC.
5. If the companion computer / PC runs ubuntu: on the Companion Computer / PC, go to network settings > Wired > settings. Go to IPv4 settings and add your IPADDR of your choising (ex: 192.168.0.6) and the NETMASK 255.255.255.0
6.If the companion computer runs rapsbian (e.g. using CM4 baseboard), refer to this [guide](https://www.ionos.com/digitalguide/server/configuration/provide-raspberry-pi-with-a-static-ip-address/) to setup a static IP for your raspberry bi.
7. Try to ping the pixhawk ip from the companion computer by running the command `ping 192.168.0.4` from the terminal.
8. Go to params in QGC and change the following settings `MAV_x_CONFIG`, where `x` is the instance number. Our default configuration uses `x=2` instance for ethernet communication.
9.  In QGroundControl, search for the `MAV_x_CONFIG` parameter.
10. Set the value of the `MAV_x_CONFIG` parameter to `ethernet`.
11. Reboot the vehicle and relaunch QGroundControl.
12. Set the value of the `MAV_x_MODE` parameter to `Custom`.
13. Set the value of the `MAV_x_RATE` parameter to `2000000` B/s.
14. Set the value of the `MAV_x_BROADCAST` parameter to `1`.
15. Set the value of the `MAV_x_RADIO_CTL` parameter to `0`.
16. Set the value of the `MAV_x_REMOTE_PRT` parameter to `14540`.
17. Set the value of the `MAV_x_UDP_PRT` parameter to `14540`.
18. Reboot the vehicle.

For using MAVROS on the companion computer to communicate with the pixhawk over ethernet, you can run the following command:
```bash
roslaunch mavros px4.launch fcu_url:=udp://@IPADDR:14540@
```
where IPADDR is the ip address of the pixhawk (by default 192.168.0.4).


##### 2. Using UART

1. Choose the MAVLink instance to work on `MAV_x_CONFIG`, where `x` is the instance number. Our default configuration uses `x=1` instance for UART communication.
2. In QGroundControl, search for the `MAV_x_CONFIG` parameter.
3. Set the value of the `MAV_x_CONFIG` parameter to the port you are using (e.g., `TELEM 2`).
4. Reboot the vehicle and relaunch QGroundControl.
5. Set the value of the `MAV_x_MODE` parameter to `Custom`.
6. Set the value of the `MAV_x_RATE` parameter to `2000000` B/s.
7. Reboot the vehicle.

After completing these steps, your MAVLink instance should be configured and ready to use.


#### Disablig landing detector
The landing detector of the PX4 can disarm the drone if not configured properly. As such, it is recommended to disable this module by setting the below parameters in QGroundControl:
1. Set the value of the `COM_DISARM_LAND` parameter to `-1`.
2. Set the value of the `MPC_THR_HOVER` parameter to a low value (e.g. `0.25`).

#### Change Frequency for navigation topics
We are interested in 3 topics for now (`VEHICLE_ANGULAR_VELOCITY`, `VEHICLE_ATTITUDE`, and `VEHICLE_LOCAL_POSITION`).

###### 1. Change the Publishing Rate in the Mavlink
First, run `uorb top -1` in the Mavlink shell to check the publishing rate of each topic. Each topic of the three mentioned above should be **at least** 200 Hz.

To change the rate of `VEHICLE_ATTITUDE` and `VEHICLE_LOCAL_POSITION`:

1. Set the parameter `IMU_INTEG_RATE` to be 400 Hz.
2. Set the parameter `EKF2_PREDICT_US` to be 2500.

Note: The parameter `IMU_INTEG_RATE` is to change the rate of the `VEHICLE_ATTITUDE` topic.

Note: The `EKF2_PREDICT_US` is to change the rate of the `VEHICLE_LOCAL_POSITION` topic, however, the topic rate will be limited to the `IMU_INTEG_RATE` parameter value.

Run `uorb top -1` in the Mavlink shell to re-check the publishing rate of the topics after applying the changes.


###### 2. Change the Stream Rate in MAVLINK_MODE_CUSTOM

1. Open `~/PX4-Autopilot/src/modules/mavlink/mavlink_main.cpp`

2. Change the three lines of the topics to be:
```cpp
configure_stream_local("VEHICLE_LOCAL_POSITION", 200.0f);
configure_stream_local("VEHICLE_ATTITUDE", 200.0f);
configure_stream_local("VEHICLE_ANGULAR_VELOCITY", 200.0f);
```

3. Save, build, and upload to the pixhawk 6x. To [build](https://docs.px4.io/main/en/flight_controller/pixhawk6c.html#building-firmware) PX4 for pixhawk 6x:

```bash
cd ~/PX4-Autopilot/
make px4_fmu-v6x_default upload
```

A successful run will end with this output:
```bash
Erase  : [====================] 100.0%
Program: [====================] 100.0%
Verify : [====================] 100.0%
Rebooting.

[100%] Built target upload
```

###### 3. Check the Topics Frequency in Mavros
1. Close QGroundControl if open.
2. Unplug and re-plug the pixhawk.
3. run:
```bash
roslaunch mavros px4.launch
```
4. Then run each topic in a separate terminal:
```bash
rostopic hz /mavros/vehicle_angular_velocity/vehicle_angular_velocity
rostopic hz /mavros/vehicle_attitude/vehicle_attitude
rostopic hz /mavros/vehicle_local_position/vehicle_local_position
```
The output should be like this:
```bash
average rate: 200.00
        min: 0.002s max: 0.019s std dev: 0.00572s window: 100
```
If the rate is successfully 200 Hz, please start from step 2 again but with 400.0f Hz.
