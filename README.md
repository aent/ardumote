# ardumote
Emulate a full remote control by using an arduino and a PPM to 2.4 GHz module.

## Interface

### Subscribed Topics
* **`/ppm`** ([ardumote/PPM])

  Set the pulse width of a certain channel.
  
### Published Topics
*None*

### Services
*None*

### Parameters
* **`port`** / std::string / Default: **/dev/ttyACM0**

  The device node under which the Arduino registers itself with the Arduino Uno.

* **`baud`** / int / Default: **115200**

  The baud rate which is used for the communication between the PC and the Arduino Uno.

* **`num_channels`** / int / Default: **5**

  The number of channels for the PPM generator library - maximum is 8.

## Installation

### Dependencies
```
sudo apt-get install arduino ros-indigo-rosserial ros-indigo-rosserial-arduino
```

### Building
```
cd ~/catkin_ws/src
https://github.com/lxrobotics/ardumote
cd ..
catkin_make
```

### Download
* Set the port under which the Arduino is connected to the PC by modifying [firmware/CMakeLists]
```
PORT /dev/ttyUSB0
```
* Download to the Arduino
```
catkin_make ardumote_firmware_ardumote-upload
```

## FAQ

* **Error**: `fatal error: string: No such file or directory` when compiling custom messages/services

**Solution**:
```
cd ~/catkin_ws/build/ardumote/
rm -rf ros_lib
cd ..
catkin_make ardumote_ros_lib
```

[ardumote/PPM]: https://github.com/lxrobotics/ardumote/blob/master/msg/PPM.msg
[firmware/CMakeLists]: https://github.com/lxrobotics/ardumote/blob/master/firmware/CMakeLists.txt
