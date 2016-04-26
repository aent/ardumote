# ardumote
Emulate a full remote control by using an arduino and a PPM to 2.4 GHz module.

## Interface

### Subscribed Topics
*None*

### Published Topics
*None*

### Services
* **`/initPPM`** ([ardumote/initPPM])

Initialize the PPM generator.

* **`/setPPM`** ([ardumote/setPPM])

The the pulse duration in us for a single channel.

### Parameters
* **`port`** / std::string / Default: **/dev/ttyACM0**

The device node under which the Arduino registers itself with the Arduino Uno.

* **`baud`** / int / Default: **115200**

The baud rate which is used for the communication between the PC and the Arduino Uno.

[ardumote/initPPM]: https://github.com/lxrobotics/ardumote/blob/master/srv/initPPM.srv
[ardumote/setPPM]: https://github.com/lxrobotics/ardumote/blob/master/srv/setPPM.srv
