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
Python 3.6  

Prerequisites
-------------
1. cv_bridge with python3 - Since ROS Melodic is built with python 2.7 but our dependencies are in python3, we must bridge this gap. To do so, follow the instructions in this link https://cyaninfinite.com/ros-cv-bridge-with-python-3/ to have a python3 cv_bridge.
2. Conda environment - Because ROS Melodic uses python 2.7, it will be easier to manage all your packages in a seperate environment with it's own python version. I reccomend using one with python 3.6 as the interpreter. Here, you should install all the dependencies of YOLOP. Here, you should also install the propoer torch version and torchvision. Specifically, follow this link https://forums.developer.nvidia.com/t/pytorch-for-jetson/72048 to install the cuda-compatible torch version and you will also most lkely need to build the corresponding torch vision from source https://forums.developer.nvidia.com/t/how-to-install-torchvision-0-9-0/187217.
3. You will also need to calibrate the ESC the first time you boot up the car. To do this, you will need to download the https://www.pololu.com/docs/0J40/3.b. This will allow you to control the SERVOs via the controller manually. Then, follow the following guide to calibrate your ESC: https://traxxas.com/support/Programming-Your-Traxxas-Electronic-Speed-Control.
4. Check if your Azure Kinect DK works. Follow the instructions here: https://learn.microsoft.com/en-us/azure/kinect-dk/set-up-azure-kinect-dk. This will allow you to use the camera. To quickly check if the camera is working, you can use $ k4aviewer.
5. Azure Kinect ROS drivers. Go to the follwoing website and follow their instructions to install/use the ros drivers: https://github.com/microsoft/Azure_Kinect_ROS_Driver.
6. You can optionally go into the YOLOP model to minimize computation. Just get rid of either driveable area or lane localization if you need one or the two. You can find it in YOLOP/lib/models/YOLOP.py

Building
--------
To build, run the following:  
source <ROS distribution>  
source <cv_bridge workspace>  
catkin_make --only-pkg-with-deps car azure_kinect_ros_driver  


Running
-------
cd ~/catkin_ws  
source devel/setup.bash  
source <cv_bridge workspace> --extend  
roslaunch launch/auto_car.launch  

You can also run the nodes seperately. You can also configure the corresponding parameters in the Azure_Kinect_ROS_Driver/launch/driver.launch and car/launch/car_image.launch files respectively

Future Notes
------------
1. Areas that have been left open have a TODO marker.
2. The camera struggloes to identify objects in the left side. I think that it relates to the hardware having the camera on the right, but more investigation needs to be done here.
3. Sometimes the camera just crashes, you will just need to unplug it and replug it to reset it.
4. I have not incorporated th driveable area or lane localization into my code. The easiest way to do this would be to incorporate that information in the PID controller.
5. If you have time, I highly reccomend porting everything over to ROS2. This will help a lot with versioning issues.

Contact
-------
If you have any more questions, feel free to reach me at rickyg@andrew.cmu.edu