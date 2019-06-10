#!/usr/bin/env python

import sys, select, termios, tty
import math
import signal
import json
# sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
import cv2
import numpy as np
import os,sys,inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(currentdir)
sys.path.insert(0,parentdir)
import airsim

import rospy
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import CompressedImage, Image

# import ros_airsim_bridge.srv as srvs

CAMERA_NAME = '0'
IMAGE_TYPE = airsim.ImageType.Scene
DECODE_EXTENSION = '.jpg'



class AirsimDrone():
    def __init__(self, name):
        # Setup Drone API Control in AirSim
        self.client = airsim.MultirotorClient()
        self.client.confirmConnection()
        self.client.enableApiControl(True, "Drone1")
        # self.client.enableApiControl(True, "Drone2")
        self.client.armDisarm(True, "Drone1")
        # self.client.armDisarm(True, "Drone2")

        self.name = name
        # self.image_pub = rospy.Publisher("/%s/image_raw/compressed" % self.name, CompressedImage)
        self.image_pub = rospy.Publisher("/%s/image_raw" % self.name, Image, queue_size=1)
        self.state_pub = rospy.Publisher("/%s/state" % self.name, PoseStamped)
        # self.image_pub2 = rospy.Publisher("/Drone2/image_raw", Image, queue_size=1)
        # self.state_pub2 = rospy.Publisher("/Drone2/state", PoseStamped)
        # self.client.takeoffAsync(vehicle_name="Drone2")
        # self.obj_loc_srv = rospy.Service('getObjectLocation', srvs.GetObjectLocation, self.getObjectLocation)

        # self.client = airsim.VehicleClient()
        # self.client.confirmConnection()
        # self.client.enableApiControl(True, name)
        # self.client.armDisarm(True, name)

        # self.name = name
        # self.image_pub = rospy.Publisher("/%s/image_raw/compressed" % self.name, CompressedImage)
        # self.state_pub = rospy.Publisher("/%s/state" % self.name, PoseStamped)

    # Begins takeoff
    def beginMovement(self):
        print("taking off")
        self.client.enableApiControl(True, self.name)
        self.client.armDisarm(True, self.name)
        return self.client.takeoffAsync(vehicle_name=self.name)

    # Publishes image from drone viewpoint
    def publishImage(self, type="color"):
        responses = self.client.simGetImages([airsim.ImageRequest("1", airsim.ImageType.Scene, False, False)])  #scene vision image in uncompressed RGB array

        for response in responses:
            img_rgb_string = response.image_data_uint8

        # Populate image message
        msg=Image() 
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "frameId"
        msg.encoding = "rgb8"
        msg.height = 360  # resolution should match values in settings.json 
        msg.width = 640
        msg.data = img_rgb_string
        msg.is_bigendian = 0
        msg.step = msg.width * 4

        # log time and size of published image
        rospy.loginfo(len(response.image_data_uint8))
        # publish image message
        # msg.data = np.array(cv2.imencode('.jpg', image_np)[1]).tostring()
        self.image_pub.publish(msg)
        # sleep until next cycle
        rospy.Rate(10).sleep()

    #     response_image = self.client.simGetImage(CAMERA_NAME, IMAGE_TYPE)
    #     np_response_image = np.asarray(bytearray(response_image), dtype="uint8")
    #     # Publish to ROS
    #     msg = CompressedImage()
    #     msg.header.stamp = rospy.Time.now()
    #     msg.format = "jpeg"
    #     msg.data = np_response_image.tostring()
    #     # msg.data = np.array(cv2.imencode('.jpg', image_np)[1]).tostring()
    #     self.image_pub.publish(msg)

    # def getObjectLocation(self, request):
    #     pose = client.simGetObjectPose(request.object_name);
    #     location = pose.position
    #     location, _ = ue_coords.transform_from_unreal(location, None)
    #     return geometry_msgs.msg.Point(x=location[0], y=location[1], z=location[2])


    # def generateVideoStream(self):
    #     while (True):
    #         # Get Image
    #         response_image = client.simGetImage(CAMERA_NAME, IMAGE_TYPE)
    #         np_response_image = np.asarray(bytearray(response_image), dtype="uint8")
    #         # Publish to ROS
    #
    #          msg = CompressedImage()
    #          msg.header.stamp = rospy.Time.now()
    #          msg.format = "jpeg"
    #          msg.data = np_response_image.tostring()
    #          # msg.data = np.array(cv2.imencode('.jpg', image_np)[1]).tostring()
    #          image_pub.publish(msg)
    #
    #         # Return Image
    #         # yield (b'--frame\r\n'
    #         #        b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n\r\n')
    #

    # Move drone using position, yaw targets
    def moveToPosition(self, pos_msg):
        # self.client.enableApiControl(True, self.name)
        # self.client.armDisarm(True, self.name)
        x, y, z = pos_msg.pose.position.x, pos_msg.pose.position.y, pos_msg.pose.position.z
        print("Move to Position")
        # print(x, y, z)
        speed = 7 # vel.x + vel.y + vel.z^2 blah
        return self.client.moveToPositionAsync(x, y, z, speed, timeout_sec=10, drivetrain=airsim.DrivetrainType.ForwardOnly, yaw_mode=airsim.YawMode(is_rate=False, yaw_or_rate=0),
                vehicle_name=self.name)
        # self.client.simSetVehiclePose(pos_msg.pose, True, '')

    # Moves drone using velocity controls
    def moveByVelocity(self, pos_msg):
        self.client.enableApiControl(True, self.name)
        self.client.armDisarm(True, self.name)

        if pos_msg.linear.z != 0:
            print(pos_msg)
        return self.client.moveByVelocityAsync(pos_msg.linear.x, pos_msg.linear.y, pos_msg.linear.z, .25,
                airsim.DrivetrainType.MaxDegreeOfFreedom,
                yaw_mode=airsim.YawMode(False,0),
                vehicle_name=self.name)

    # Publishes current position and orientation of drone
    def publishState(self):
        drone_state = self.client.getMultirotorState(vehicle_name=self.name)
        # drone_state = self.client.simGetVehiclePose()
        pos_ned = drone_state.kinematics_estimated.position
        orientation_ned = drone_state.kinematics_estimated.orientation
        pos = airsim.Vector3r(  pos_ned.x_val, pos_ned.y_val, pos_ned.z_val)
        orientation = airsim.Quaternionr( orientation_ned.w_val,
orientation_ned.z_val, orientation_ned.x_val, orientation_ned.y_val)

        sim_pose_msg = PoseStamped()
        sim_pose_msg.pose.position.x = pos.x_val
        sim_pose_msg.pose.position.y = pos.y_val
        sim_pose_msg.pose.position.z = pos.z_val
        sim_pose_msg.pose.orientation.w = orientation.w_val
        sim_pose_msg.pose.orientation.x = orientation.x_val
        sim_pose_msg.pose.orientation.y = orientation.y_val
        sim_pose_msg.pose.orientation.z = orientation.z_val
        self.state_pub.publish(sim_pose_msg)
        # sim_pose_msg.header.seq = 1
        # sim_pose_msg.header.frame_id = "world"

        # sim_pose_msg = PoseStamped()
        # sim_pose_msg.pose.position.x = drone_state.position.x_val
        # sim_pose_msg.pose.position.y = drone_state.position.y_val
        # sim_pose_msg.pose.position.z = drone_state.position.z_val
        # sim_pose_msg.pose.orientation.w = drone_state.orientation.w_val
        # sim_pose_msg.pose.orientation.x = drone_state.orientation.x_val
        # sim_pose_msg.pose.orientation.y = drone_state.orientation.y_val
        # sim_pose_msg.pose.orientation.z = drone_state.orientation.z_val
        # self.state_pub.publish(sim_pose_msg)


class AirSimCar():
    def __init__(self, name="Car1"):
        self.client = airsim.CarClient()
        self.client.confirmConnection()
        self.client.enableApiControl(True, name)
        self.client.armDisarm(True)

        self.name = name
        self.image_pub = rospy.Publisher("/%s/output/image_raw/compressed" % self.name, CompressedImage)

    def moveToPosition(self, pos_msg=""):
        self.client.enableApiControl(True, name)
        self.client.armDisarm(True, name)
        speed = 1000 # vel.x + vel.y + vel.z^2 blah
        return self.client.moveToPositionAsync(0, 0, 0, speed, vehicle_name=name)

    def publishImage(self, type="color"):
        response_image = self.client.simGetImage(CAMERA_NAME, IMAGE_TYPE)
        np_response_image = np.asarray(bytearray(response_image), dtype="uint8")
        # Publish to ROS
        msg = CompressedImage()
        msg.header.stamp = rospy.Time.now()
        msg.format = "jpeg"
        msg.data = np_response_image.tostring()
        # msg.data = np.array(cv2.imencode('.jpg', image_np)[1]).tostring()
        self.image_pub.publish(msg)

def parseSettings(json_loc):
    with open(json_loc) as json_file:
        data = json.load(json_file)
        return data
    return None
