
High level code
===========================

.. meta::
   :description lang=en: info about Code.
   
Matlab version used R2018a
Robotic toolbox version: Robot-10.3.1

High level code does all calculations and simulations. It will usually run on the PC since it will be doing a lot of
math and matrix multiplications. It will use some kind of inverse kinematics solver that will generate joint angles and speeds.
Again, gross simplification, but we will talk more about it later.

All code is in github repository.

Setting up Matlab
------------------
I used Robot-10.3.1 library version from Peter Corke. You can find it here: https://petercorke.com/resources/downloads/ 
You need to install that libary to use codes used in this documentation.


Matlab code for testing in live script
---------------------------------------

File is located at: xxxx

It works together with C/C++ Arduino file located at: xxxx

When you run this live script it allows you to control robot using sliders and test all joints that way.
It also shows simulated robot in Matlab window.Script will give back error if connection is not made with robot arm.

.. code-block:: python
   :linenos:
   
   #Here you need to change your COM port to one your arm is connected to
   delete(instrfind);
   x=serial('COM12','BaudRate', 9600);
   fopen(x)
   
   
Here we Create model of robot arm. If you are using gripper on robot change Gripping point to your value.

.. code-block:: python
   :linenos:
   
   gripping_point = 0.057 ;
   L(1) = Link([0 0.23682 0 pi/2]);
   L(2) = Link([0 0 0.32 0]);
   L(3) = Link([0 0 0.0735 pi/2 ]);
   L(4) = Link([0 0.2507 0 -pi/2 ]);
   L(5) = Link([0 0 0 pi/2]);
   L(6) = Link([0 gripping_point 0 0]);
   robot = SerialLink(L);
   robot.name='faze4';

Now if you dont want to send data to robot you can make print in code bellow = 0.
You would do this if you want to adjust to your robots zero position or move all joints at once.

.. code-block:: python
   :linenos:
   
   if(print == 1)
   fwrite(x,Start,'char'); 
   fwrite(x,Start,'char'); 
   for i = 1:6    

Matlab trajectory planning
----------------------------

ROS  
----
