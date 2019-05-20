# ros_airsim_bridge

How to use:
'''
$ roslaunch airsim_bridge ros_airsim.launch
'''

### airsim_bridge_server.py
The main bridge between the AirSim plugin running in the Unreal instance and ROS.

### airsim_objects.py
Contains implementations of the objects used in airsim_bridge_server.py. The AirsimDrone class is the main focus here, as we aren't focusing on development for the car (inside of AirSim).

### unreal_cv_image_server.py
Adapted from https://github.com/jskinn/unrealcv-ros
Publishes camera feeds in areas not visible to the drone. Make sure the port in the unrealcv.ini config file is set to 9234.
