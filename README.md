# Project-RASDOF
[Moving a 6 DOF robot arm using hand gestures via a LEAP Motion Controller]


[What does each file do]

There are three main Python files in this project:

1) ArduinoController
This file is a class that helps the user automatically determine what COM port the Arduino Uno is currently connected to. This class also includes standard functions for serial communication such as writing data, opening and closing the serial connection.

2) get_leap_data
This file

3) kinematics_calculations


[How do the files work with each other]

get_leap_data acts as the main file. At the start of the script, it initialises the serial connection by calling the ArduinoController class. With the data received from the LEAP motion controller, these raw data are then parsed into the kinematics_calculations file to obtain the motor angles for each servo motor on the robot arm. These processed data is then written to an Arduino Uno. The Arduino Uno reads the data and writes the motor angles to the respective servos to move the robot arm.
