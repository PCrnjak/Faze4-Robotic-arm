
Low level code
===========================

.. meta::
   :description lang=en: info about Code.
   
Code for this robotic arm can be devided in 2 parts:
* Low level code
* High level code

Low level code is code that runs on our microcontroller (in our case teensy 3.5).
Its main purpose is to receive information from high level code and transform that information 
into useful signals for stepper drivers. That is a gross simplification, but codes used 
will be explained in more detail later.

High level code does all calculations and simulations. It will usually run on the PC since it will be doing a lot of
math and matrix multiplications. It will use some kind of inverse kinematics solver that will generate joint angles and speeds.
Again, gross simplification, but we will talk more about it later.

All code is in github repository.

Tennsy environment setup
------------------------

Teensy low level code for testing in Matlab(live script)
--------------------------------------------------------

Teensy low level code for Matlab trajectory planning
----------------------------------------------------

Teensy low level code for ROS 
-----------------------------
