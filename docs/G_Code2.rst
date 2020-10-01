
High level code
===========================

.. meta::
   :description lang=en: info about Code.
   
Matlab version used R2018a
Robotic toolbox version: Robot-10.3.1

These codes are best used to take parts from them to develop your own code or just to test the robot.
They are clunky and you must be really careful when using them on your robot.

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

File is located at: Faze4-Robotic-arm/Software1/High_Level_Matlab/GUI_Matlab.mlx

It works together with C/C++ Arduino file located at: Faze4-Robotic-arm/Software1/Low_Level_Arduino/Arduino_GUI_code.ino

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

Files are located in: Faze4-Robotic-arm/Software1/High_Level_Matlab/Trajectory_Matlab/
C/C++ Arduino files are in: Faze4-Robotic-arm/Software1/Low_Level_Arduino/Robot_Arduino_trajectory/

As seen in this video: https://www.youtube.com/watch?v=0We33vvnHSE&t=6s&ab_channel=PetarCrnjak

Code is used to make a trajectory that arm will follow.
Structure of the code is devided in logical sub files:

* Robot_angular_vel.mlx
* Robot_calculate_angles.mlx
* Robot_ik_code_1.mlx
* Robot_path.mlx
* Robot_plot_angles_velocity.mlx
* Robot_sending.m
* Robot_setup.mlx
* Robot_simulation.m
* Robot_trajectory.mlx

Robot_ik_code_1.mlx is main file. From there you can remove and add what stepts to do.
For example if you dont want to send data to robot just comment out Robot_sending.
If you dont want to simulate robot comment out Robot_simulation...




ROS  
----
