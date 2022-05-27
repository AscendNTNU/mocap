# mocap_node
ROS (1) node for relaying mocap data from ROS to the drone over MAVlink.  

## Prerequisites
- Pymavlink is installed: https://github.com/ArduPilot/pymavlink
- ROS is installed (only tested with noetic): http://wiki.ros.org/noetic/Installation/Ubuntu 
- MAVProxy is installed: https://ardupilot.org/mavproxy/docs/getting_started/download_and_installation.html
- For mocap to work you need to upload these parameters: https://github.com/AscendNTNU/Drone-parameters/blob/main/mocap.params

## How to use the system 
### On the Ubuntu computer
Start a roscore 

### Qualisys (Windows 10) 
- Login with PIN 0052
- Start the Qualisys software and choose the QualisysAscend project.
- Define a body and start recording.  
- Open a terminal and start by sourcing the local workspace setup.
```
devel\setup.bat
```
Set the ROS_MASTER_URI
```
setx ROS_MASTER_URI "http://xxx.xxx.x.xxx:xxxxx"
```
Run the qualisys node 
```
roslaunch mocap_qualisys qualisys.launch
```

### On the Ubuntu computer 
Launch mavrpoxy where you installed it
```
./mavproxy --master=/dev/ttyUSB0 --out=udp:127.0.0.1:14550 --out=udp:127.0.0.1:14551 --out=udp:127.0.0.1:14552
```
mocap_node uses ```udp:127.0.0.1:14551``` as default. That means ```udp:127.0.0.1:14550``` is free for for example QGroundControl, and ```udp:127.0.0.1:14552``` is free for other stuff.

Go in to your ROS workspace and launch mocap_node. Remember to source both ros and ```devel/setup.bash```.
```
roslaunch mocap_node mocap.launch
```
Now you should be able to arm and take off. 

# Other considerations
The [```ATT_POS_MOCAP``` message](https://mavlink.io/en/messages/common.html#ATT_POS_MOCAP) contains a ```time_usec``` field which needs to be filled. This field is filled with local time on the computer running ```mocap_node```. To compensate for [Qualisys processing delay](https://www.qualisys.com/news/real-time-latency-tests-of-a-qualisys-system-in-the-sensory-motor-systems-lab-at-eth-zurich-switzerland/) of 6-7 ms and the transmission delay from the computer running Qualisys to the computer running ```mocap_node```, [```VISO_DELAY_MS```](https://ardupilot.org/copter/docs/parameters.html#viso-delay-ms-visual-odometry-sensor-delay) is set to 10 ms. Here we assumed that the flight controller is synced with the ```mocap_node``` computer. It is believed that this is done thorugh Mavproxy.

There is no need to set origin and home explicitly. But if you have to, check the scripts folder.

