#!/usr/bin/env python

# Example ROS node for publishing AirSim images.

# AirSim Python API
import setup_path 
import airsim
import sys, select, termios, tty, signal

import rospy

# ROS Image message
from sensor_msgs.msg import Image
from std_msgs.msg import Int16


class airpub():
    def __init__(self):
        pub = rospy.Publisher("Drone1/image_raw", Image, queue_size=1)
        
        rospy.init_node('image_raw', anonymous=True)
        rate = rospy.Rate(10) # 10hz
        rospy.Subscriber("/Camera_Num", Int16, self.setCamNum)

        self.camCoords = [[10, 10, -2], [0, 10, -2], [10, 0, -2], [5, 5, -5]]

        # connect to the AirSim simulator 
        self.client = airsim.MultirotorClient()
        self.client.confirmConnection()
        self.client.enableApiControl(True, "Drone1")
        self.client.enableApiControl(True, "Camera1")
        self.client.armDisarm(True, "Drone1")
        self.client.armDisarm(True, "Camera1")
        self.camNum = 0

        self.pose = self.client.simGetVehiclePose()

        while not rospy.is_shutdown():
            # get camera images from the car
            print(self.camNum)
            # client.simPause(False)
            if self.camNum is 4:
                print("Drone")
                # responses1 = client.simGetImage("0", airsim.ImageType.Scene, "Drone1")
                # img_rgba_string = responses1
                responses1 = self.client.simGetImages([
                    airsim.ImageRequest("0", airsim.ImageType.DepthVis),  #depth visualization image
                    airsim.ImageRequest("1", airsim.ImageType.Scene, False, False)], vehicle_name="Drone1")  #scene vision image in uncompressed RGB array
            else:
                print("Camera")
                responses1 = self.client.simGetImages([
                    airsim.ImageRequest("0", airsim.ImageType.DepthVis),  #depth visualization image
                    airsim.ImageRequest("1", airsim.ImageType.Scene, False, False)], vehicle_name="Camera1")  #scene vision image in uncompressed RGB array
         
            for response1 in responses1:
                img_rgba_string = response1.image_data_uint8

            # Populate image message
            msg=Image() 
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = "frameId"
            msg.encoding = "rgba8"
            msg.height = 240  # resolution should match values in settings.json 
            msg.width = 480
            msg.data = img_rgba_string
            msg.is_bigendian = 0
            msg.step = msg.width * 4

            # log time and size of published image
            # rospy.loginfo(len(response1.image_data_uint8))
            # publish image message
            pub.publish(msg)
            rate.sleep()

    def setCamNum(self, num):
        # self.client.confirmConnection()
        # self.client.enableApiControl(True, "Camera1")
        # self.client.armDisarm(True, "Camera1")

        self.camNum = num.data
        
        if self.camNum is not 4:


            self.pose.position.x_val = self.camCoords[self.camNum][0]
            self.pose.position.y_val = self.camCoords[self.camNum][1]
            self.pose.position.z_val = self.camCoords[self.camNum][2]

            self.client.simSetVehiclePose(self.pose, True, "Camera1")
        


if __name__ == '__main__':
    try:
        airpub()
    except rospy.ROSInterruptException:
        pass
