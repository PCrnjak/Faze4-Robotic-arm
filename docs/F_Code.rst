
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


In first 100 lines are pin definitins. They are made for first version of faze4 distribution board that is now legacy and is replaced with second version that is also on sale here: 

When power is applied to robot it will first perform homing routine. After all joints are homed it will go to starting/start by postion. After that it is ready to receive data from Code located in: Faze4-Robotic-arm/Software1/High_Level_Matlab/GUI_Matlab.mlx

Program is actually really simple. After homing we get data from matlab or any other source and then move robot according to that data.
As you can see in code bellow we call only 2 functions in main loop.

.. code-block:: python
   :linenos:
   
   void loop() {

   while (error == 0) {

    get_data();
    move_all();
  }
}
   
But before we get to main loop robot needs to home. Now if robot starts moving away from its limit switches during homing you need to stop it and do one of 2 things:

* Switch stepper motor phases 
* change direction in software

After robot homes it goes to standy/start position.
***+slika******

If you are writing your own code this is useful to steal ;)
In main loop move_all(); is based on functions like this:

.. code-block:: python
   :linenos:
   
    if (needed_position[joint[1]] > current_position[joint[1]] ) {
    digitalWrite(dirPin[joint[1]], LOW);
    move_routine_forward(joint[1]);

    }
    else if (needed_position[joint[1]] < current_position[joint[1]] ) {
    digitalWrite(dirPin[joint[1]], HIGH);
    move_routine_backward(joint[1]);
    }
      
      
What this snipet does is this: Code is always looping thru this code in main loop.
If needed_position is same as current_position nothing happens. If it is different tho we check if it is smaller or bigger and on those terms move_forward or move_backward
When we perform this for each of 6 motors of robot arm we have really good controler.

Move routine works like this:

.. code-block:: python
   :linenos:
   
   void move_routine_forward(int joint_num) {
   currentMillis = micros();
   #state is used to prevent triggering of this if statement twice in row
   #same goes for second one , this secures perfect square wave form
   if (currentMillis - previousMillis[joint_num] >= current_pulse_widht[joint_num] and state[joint_num] == 0) {
    previousMillis[joint_num] = currentMillis;
    digitalWrite(stepPin[joint_num], HIGH);
    state[joint_num] = 1;
    }
      else if (currentMillis - previousMillis[joint_num] >= (current_pulse_widht[joint_num]) and state[joint_num] == 1) {
    previousMillis[joint_num] = currentMillis;
    digitalWrite(stepPin[joint_num], LOW);
    state[joint_num] = 0;
    current_position[joint_num] = current_position[joint_num] + 1;

   }

   }
    
   
One step of stepper motor is defined by change from high to low signal on step pin. speed is defined by length of that signal.
In this code we use micros as timer function. Lets say we want to half period of our pulse is 500 us. Once we see 500us or more passed we switch step pin to HIGH and move state variable to low. We move that variable low as an indicator that next 500 us will swtich pin to LOW. Now when we switch to LOW state goes to 0 and we increment current position +1. This proces goes until  move_all(); see that current_position = needed_position.



Teensy low level code for Matlab trajectory planning
----------------------------------------------------

Teensy low level code for ROS 
-----------------------------
