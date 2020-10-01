
Low level code
===========================

.. meta::
   :description lang=en: info about Code.
   
Code is under MIT license: https://github.com/PCrnjak/Faze4-Robotic-arm/blob/master/LICENSE
Matlab version used R2018a
Robotic toolbox version: Robot-10.3.1
Teensyduino version: Teensyduino, Version 1.53
Teensy board used: 3.5
Arduino IDE version: 1.8.13


These codes are best used to take parts from them to develop your own code or just to test the robot.
They are clunky and you must be really careful when using them on your robot.
   
Code for this robotic arm can be devided in 2 parts:

* Low level code
* High level code

Low level code is code that runs on our microcontroller (in our case teensy 3.5).
Its main purpose is to receive information from high level code and transform that information 
into useful signals for stepper drivers. That is a gross simplification, but codes used 
will be explained in more detail below.

All code is in github repository.

Tennsy environment setup
-------------------------

Install arduino IDE: https://www.arduino.cc/en/main/software

Download Teensyduino from here: Teensyduino is a software add-on for the Arduino software.
Teensyduino is a software add-on for the Arduino software. 


Teensy low level code for testing in Matlab(live script)
--------------------------------------------------------

File is located at: Faze4-Robotic-arm/Software1/Low_Level_Arduino/Arduino_GUI_code.ino

It works together with Matlab code located at: Faze4-Robotic-arm/Software1/High_Level_Matlab/GUI_Matlab.mlx


Teensy low level code for Matlab trajectory planning
----------------------------------------------------

In first 100 lines are pin definitins. They are made for first version of faze4 distribution board that is now legacy and is replaced with second version that is also on sale here: 

When power is applied to robot it will first perform homing routine. After all joints are homed it will go to starting/stand by postion. After that it is ready to receive data from Code located in: Faze4-Robotic-arm/Software1/High_Level_Matlab/GUI_Matlab.mlx

Teensy low level code for ROS 
-----------------------------
