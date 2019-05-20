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
    },
    "CommandMode": "Velocity" for velocity based movement, "Position" for position based movement
}

"""

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
        drone_name = "Drone1" #settings["Vehicles"][0]
        # car_name = "Car1"

        # setup ROS node
        rospy.init_node('airsim_bridge')
        rospy.Subscriber("/%s/cmd_vel" % drone_name, Twist, self.droneTwistCallback)
        rospy.Subscriber("/%s/pose" % drone_name, PoseStamped, self.dronePoseCallback)
        # rospy.Subscriber("/%s/cmd_vel" % car_name, Twist, carTwistCallback)
        self.r = rospy.Rate(4) # 4hz

        # setup vehicles
        self.drone = airsim_objects.AirsimDrone(drone_name)
        self.drone.beginMovement().join()


    def run(self):
        # Publish camera/state info
        self.drone.publishImage()
        self.drone.publishState()


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
