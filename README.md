# agrorob packages
- **agrorob_gps:** ublox gps driver and ntrip client
- **agrorob_launch:** launches all necessary packages on robot
- **agrorob_wiki:** Installation and configuration instructions, 


# configuration
## Install dependencies
```
$ rosdep install --from-paths src -y --ignore-src
$ sudo apt-get install ros-humble-teleop-twist-joy
```


## Bring-up CAN NIC
```$ sudo ip link set can0 up type can bitrate 250000```

# Running packages separately
## Can to ros bridge
```$ ros2 launch ros2_socketcan socket_can_bridge.launch.xml ```    