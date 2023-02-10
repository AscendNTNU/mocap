# mocap_node
ROS (1) node for relaying mocap data from ROS to the drone over MAVlink.  

## Prerequisites
- Pymavlink is installed: https://github.com/ArduPilot/pymavlink
- ROS is installed (only tested with noetic): http://wiki.ros.org/noetic/Installation/Ubuntu 
- MAVProxy is installed: https://ardupilot.org/mavproxy/docs/getting_started/download_and_installation.html
- For mocap to work you need to upload these parameters: https://github.com/AscendNTNU/Drone-parameters/blob/main/mocap.params
- Both your Ubuntu computer and the qualisys computer is connected to the ascend network

## How to use the system 
### On the Ubuntu computer
Source ros and start a roscore
```
source /opt/ros/noetic/setup.bash
roscore
```

### Qualisys (Windows 10) 
- Login with PIN 0052
- Start the Qualisys software and choose the QualisysAscend project.
- Start recording (click red dot) and click "cancel" to open the 3D display.
- If the drone is not already defined in Qualisys do it by selecting the dots on the drone, right click one of them and create a new body. When doing this the drone should point in the direction of the x-axis. Give the drone a suiting name.
- Open a terminal by clicking on the terminal icon in the taskbar.
- Source the local workspace setup with the following command
```
devel\setup.bat
```
Set the ROS_MASTER_URI to the ip of the Ubuntu pc (probably 192.168.0.115) and port 11311
```
setx ROS_MASTER_URI "http://192.168.0.115:11311"
```
Set the ROS_IP to the ip of the Windows pc (probably 192.168.0.100)
```
setx ROS_IP "192.168.0.100"
```
Run the qualisys node 
```
roslaunch mocap_qualisys qualisys.launch
```

### On the Ubuntu computer 
At this point the drone should be powered on and the telemetry radio should be connected to the Ubuntu pc

Launch mavproxy where you installed it (probably in home) (make sure QGroundControl is not running when launching mavproxy)
```
mavproxy --master=/dev/ttyUSB0 --out=udp:127.0.0.1:14550 --out=udp:127.0.0.1:14551 --out=udp:127.0.0.1:14552
```
If this doesn't work you can try changing ```ttyUSB0``` to ```ttyUSB1```

mocap_node uses ```udp:127.0.0.1:14551``` as default. That means ```udp:127.0.0.1:14550``` is free for for example QGroundControl, and ```udp:127.0.0.1:14552``` is free for other stuff.

Go in to your ROS workspace (```ws```) and launch mocap_node. Remember to source both ros and ```devel/setup.bash``` first.
```
roslaunch mocap_node mocap.launch drone:=nostromo
```
Change ```nostromo``` with the name you gave the drone.

Now you should be able to arm and take off. 

# Other considerations
The [```ATT_POS_MOCAP``` message](https://mavlink.io/en/messages/common.html#ATT_POS_MOCAP) contains a ```time_usec``` field which needs to be filled. This field is filled with local time on the computer running ```mocap_node```. To compensate for [Qualisys processing delay](https://www.qualisys.com/news/real-time-latency-tests-of-a-qualisys-system-in-the-sensory-motor-systems-lab-at-eth-zurich-switzerland/) of 6-7 ms and the transmission delay from the computer running Qualisys to the computer running ```mocap_node```, [```VISO_DELAY_MS```](https://ardupilot.org/copter/docs/parameters.html#viso-delay-ms-visual-odometry-sensor-delay) is set to 10 ms. Here we assumed that the flight controller is synced with the ```mocap_node``` computer. It is believed that this is done thorugh Mavproxy.

There is no need to set origin and home explicitly. But if you have to, check the scripts folder.
