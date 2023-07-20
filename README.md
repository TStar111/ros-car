# ros-car

Module desrcibing the ROS interface for the development of a semi-autonomous robotic car.

HARDWARE
--------
Traxxas RC Car  
Nvidia Jetson Xavier  
Azure Kinect Sensor  
ESC XL5  
Maestro Micro  

SOFTWARE
--------
ROS Melodic  
Ubuntu 18.04  
Python 3.9  

Notes
-----
This package depends on the build of the cv_bridge package in python3. However, since ROS Melodic is built with python 2.7, you will need to build it separately from the rest of the car package.Then, you will need to source extend it so you can use it.