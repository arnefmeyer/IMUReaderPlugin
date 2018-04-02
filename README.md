# IMUReader

Plugin for open-ephys [plugin-GUI](https://github.com/open-ephys/plugin-GUI/) to control and read data from an interia measurement unit (IMU) via an Arduino, Teensy, or another compatible microcontroller. Communication is done via i2c. Currently the following IMUs are supported:

* **LSM9DS0** the sketch can be found in "LSM0DS0" folder. Requires the Adafruit LSM9DS0 and Unified Sensor libraries. Both can be installed via the Arduino/Teensyduino IDE (Sketch -> Include Library -> Manage Libraries). For details see [here](https://learn.adafruit.com/adafruit-lsm9ds0-accelerometer-gyro-magnetometer-9-dof-breakouts/overview).
* **MPU9250** see "MPU9250" folder. No external libraries required. The current version is optimized for a Teensy but can easily be update to work with an Arduino (see comments in sketch). Hardware setup:  
	| Sensor | Arduino/Teensy pin  
	-------------------------------  
	| SCL &nbsp;&nbsp;&nbsp;  | 19  
	| SDA &nbsp;&nbsp;&nbsp;   | 18  
	| VCC &nbsp;&nbsp;   | 3.3V   
	| GND &nbsp;&nbsp;   | GND  
 

## Dependencies

This plugin requires the following parts

- Arduino/Teensy sketches (see above)


## Installation

Copy the IMUReader folder to the plugin folder of your GUI. Then build the all plugins as described in the [wiki](https://open-ephys.atlassian.net/wiki/display/OEW/Linux).


## Trigger signal

The Arduino/Teensy is sending a trigger pulse (~1 ms) on pin 13 (LED pin) for each recorded frame. Connect this pin (+ GND) to one of the digital inputs of the open-ephys acquisition board (e.g., via an I/O board).


## Remarks

Note that due to the i2c standard the cable length is limited to a few meters. In my experience 1.5 meters work without any problems.


## Reading/saving data without open-ephys GUI

The Arduin/Teensy sketches can also be used without open-ephys GUI. An example script for controlling the IMU and reading/saving data can be found in the "Python" folder.

