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

[ardumote/PPM]: https://github.com/lxrobotics/ardumote/blob/master/msg/PPM.msg
