#!/usr/bin/env python

import setup_path 
import airsim
import rospy
from geometry_msgs.msg import PoseStamped

import sys
import time

from harps_interface.msg import *
from std_msgs.msg import Int16

z = -8
velocity = 20

def stop(msg):
    client.moveByVelocityAsync(0, 0, 0, 10, vehicle_name="Drone1")

def movetoGoal(msg):

    #print("Callback")
    #print(msg,msg.x,msg.y); 
    print("Goal Recieved: {}, {}".format(msg.x,msg.y))
    reached_pub = rospy.Publisher("/GoalReached", Int16, queue_size=1)
    move_msg = Int16(); 
    move_msg.data = 0; 
    reached_pub.publish(move_msg); 
    path = []

    tmpZ = z; 
    for i in range(0, len(msg.x)):
        print(msg.x[i])
        print(msg.y[i])
        if(msg.x[i] > 500 and msg.y[i] < 400):
        	tmpZ = -14;
        path.append(airsim.Vector3r(msg.x[i], msg.y[i], tmpZ))

    client.moveOnPathAsync(path, velocity, 2000, airsim.DrivetrainType.ForwardOnly, 
                airsim.YawMode(False,0), velocity + (velocity/2), 1, vehicle_name="Drone1")
    
    # x, y = msg.x, msg.y

    # print("Moving to: ", x,y)

    # path = []

    # path.append(airsim.Vector3r(x, y, z))

    # client.moveToPositionAsync(x, y, z, velocity, 2000,airsim.DrivetrainType.ForwardOnly, airsim.YawMode(False,0), velocity + (velocity/2), 1, vehicle_name="Drone1")




if __name__ == "__main__":
    
    # velocity = 15
    # z = -30
	
    rospy.Subscriber("Drone1/Goal", path, movetoGoal)
    rospy.Subscriber("Drone1/Stop", Int16, stop)
    rospy.init_node("move_goal")
    rate = rospy.Rate(10)

    client = airsim.MultirotorClient()
    client.confirmConnection()
    client.enableApiControl(True, vehicle_name="Drone1")
    #client.simEnableWeather(True); 
    client.client.call('simEnableWeather',True); 
    client.client.call('simSetWeatherParameter',0,.9) #Rain
    client.client.call('simSetWeatherParameter',2,.9) #Snow
    client.client.call('simSetWeatherParameter',7,.9) #Fog
    client.client.call('simSetWeatherParameter',4,.1) #MapleLeaf
    client.client.call('simSetWeatherParameter',3,.9) #RoadSnow
    client.client.call('simSetWeatherParameter',6,.9) #dust


    # client.simSetWeatherParameter(airsim.WeatherParameter.Rain,1)
    # client.simSetWeatherParameter(airsim.WeatherParameter.Snow,1)
    # client.simSetWeatherParameter(airsim.WeatherParameter.Fog,1)
    # client.simSetWeatherParameter(airsim.WeatherParameter.MapleLeaf,1)
    # client.simSetWeatherParameter(airsim.WeatherParameter.RoadSnow,1)

    # client = airsim.VehicleClient()
    client.confirmConnection()
    client.enableApiControl(True, vehicle_name="Drone1")
    client.armDisarm(True)

    landed = client.getMultirotorState(vehicle_name="Drone1").landed_state
    if landed == airsim.LandedState.Landed:
        print("taking off...")
        client.takeoffAsync(vehicle_name="Drone1").join()

    landed = client.getMultirotorState(vehicle_name="Drone1").landed_state
    if landed == airsim.LandedState.Landed:
        print("takeoff failed - check Unreal message log for details")

    print("climbing to altitude: {} meters".format(-z)); 
    client.moveToPositionAsync(0, 0, z, velocity, vehicle_name="Drone1").join()
    print("Ready for Goals")



    # # start = client.getMultirotorState(vehicle_name="Drone1")
    # # print(start)

    # print("Going to (10, 0)")
    # # client.moveToPositionAsync(100, 0, z, velocity, vehicle_name="Drone1").join()



    while not rospy.is_shutdown():

        rospy.spin()
        
        # x = float(input("x: "))
        # y = float(input("y: "))

        # print("Going to (", x,",", y,")")
        # client.moveToPositionAsync(x, y, z, velocity, 2000,airsim.DrivetrainType.ForwardOnly, airsim.YawMode(False,0), velocity + (velocity/2), 1, vehicle_name="Drone1")

    # print("Going to (0, 0)")
    # client.moveToPositionAsync(100, 0, z, velocity,  2000,airsim.DrivetrainType.ForwardOnly, airsim.YawMode(False,0), velocity + (velocity/2), 1,  vehicle_name="Drone1").join()

    path = []

    path.append(airsim.Vector3r(100, 0, z))
    path.append(airsim.Vector3r(0, 0, z))

    # client.moveOnPathAsync(path, velocity, airsim.DrivetrainType.ForwardOnly, airsim.YawMode(False,0), vehicle_name="Drone1").join()

    # client.moveOnPathAsync(path, velocity, 2000, airsim.DrivetrainType.ForwardOnly, 
    #             airsim.YawMode(False,0), velocity + (velocity/2), 1, vehicle_name="Drone1").join()
