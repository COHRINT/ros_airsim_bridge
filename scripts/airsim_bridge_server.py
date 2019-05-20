#!/usr/bin/env python

# Use below in settings.json
"""
{
    ...
    "Vehicles": {
        "Drone1": {
          "VehicleType": "SimpleFlight",
          "X": 0, "Y": 0, "Z": -5,
          "Yaw": 90
        }
        "Car1": {
          "VehicleType": "PhysXCar",
          "X": 0, "Y": 0, "Z": -2
        }
    }
}

"""

# Add visualization_msgs/Marker for adding points in Unreal (or getting points/boundaries from unreal)

import sys, select, termios, tty
import math
import signal

import rospy
from geometry_msgs.msg import Twist, PoseStamped

import airsim_objects

def main():
    ab = airsim_bridge()
    while not rospy.is_shutdown():
        ab.run()
        rospy.Rate(4).sleep()

class airsim_bridge():
    def __init__(self):
        # get airsim settings
        settings = airsim_objects.parseSettings("/home/tetsuo/Documents/AirSim/settings.json")
        com_mode = settings["CommandMode"]
        drone_name = "Drone1" #settings["drone_name"] =
        car_name = "Car1" #settings["car_name"] =

        # setup ROS node
        rospy.init_node('airsim_bridge')
        rospy.Subscriber("/%s/cmd_vel" % drone_name, Twist, self.droneTwistCallback)
        rospy.Subscriber("/%s/pose" % drone_name, PoseStamped, self.dronePoseCallback)
        # rospy.Subscriber("/%s/cmd_vel" % car_name, Twist, carTwistCallback)
        self.r = rospy.Rate(4) # 4hz

        # setup vehicles
        self.drone = airsim_objects.AirsimDrone(drone_name)
        self.drone.beginMovement().join()
        # droneTwist.linear.x = 0; droneTwist.linear.y = 0; droneTwist.linear.z = 0
        # droneTwist.angular.x = 0; droneTwist.angular.y = 0; droneTwist.angular.z = 0
        # car = airsim_objects.AirSimCar(settings["car_name"])
        # carTwist.linear.x = 0; carTwist.linear.y = 0; carTwist.linear.z = 0
        # carTwist.angular.x = 0; carTwist.angular.y = 0; carTwist.angular.z = 0

        # begin

        # print("poop")


    def run(self):
        # Publish camera/state info
        self.drone.publishImage()
        self.drone.publishState()

        # Move vehicle
        # if com_mode == 'Velocity':
        #     f1 = drone.moveByVelocity(droneTwist)
        # elif com_mode == 'Position':
        #     if doneMoving
        #         f1 = drone.moveToPosition(dronePose)
        # else:
        #     print("Please select valid CommandMode ('Velocity' or 'Position') in settings.json file")

        # f1.join()
        # f2.join()


    def droneTwistCallback(self, data):
        droneTwist = data
        self.drone.moveByVelocity(droneTwist)

    def dronePoseCallback(self, data):
        self.drone.moveToPosition(data)


# def carTwistCallback(data):
#     carTwist = data


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
