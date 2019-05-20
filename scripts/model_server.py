#!/usr/bin/env python

##### DO NOT USE!! ALREADY IMPLEMENTED IN airsim_bridge_server.py


import sys, select, termios, tty
import math
import signal

import rospy
from geometry_msgs.msg import Twist

import pprint

import airsim_objects

class modelServer():
    def __init__(self):
        # get airsim settings
        settings = airsim_objects.parseSettings("/home/tetsuo/Documents/AirSim/settings.json")
        print(settings)
        drone_name = "Drone1"

        # setup drone connection
        self.client = airsim.MultirotorClient()
        self.client.confirmConnection()
        self.client.enableApiControl(True, drone_name)
        self.client.armDisarm(True, drone_name)

        # setup ROS node
        rospy.init_node('model_server')
        # rospy.Subscriber("/%s/cmd_vel" % drone_name, Twist, self.droneTwistCallback)

        # s = rospy.Service('getModel', GetModel, self.handle_get_model)

        pose1 = self.client.simGetObjectPose("Fence_2");
        print("OrangeBall - Position: %s, Orientation: %s" % (pprint.pformat(pose1.position),
        pprint.pformat(pose1.orientation)))

        rospy.spin()

    def handle_get_model(self, req):
        pose1 = client.simGetObjectPose("Fence_2");
        print("OrangeBall - Position: %s, Orientation: %s" % (pprint.pformat(pose1.position),
        pprint.pformat(pose1.orientation)))

        # begin
        # while not rospy.is_shutdown():
        #     # Publish camera/state info
        #     self.drone.publishImage()
        #     self.drone.publishState()
        #
        #     r.sleep()


# def carTwistCallback(data):
#     carTwist = data


if __name__ == '__main__':
    try:
        modelServer()
    except rospy.ROSInterruptException:
        pass
