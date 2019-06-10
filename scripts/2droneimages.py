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
        # pub2 = rospy.Publisher("Camera1/image_raw", Image, queue_size=1)
        # pub3 = rospy.Publisher("Camera2/image_raw", Image, queue_size=1)
        # pub4 = rospy.Publisher("Camera3/image_raw", Image, queue_size=1)
        # pub5 = rospy.Publisher("Camera4/image_raw", Image, queue_size=1)
        rospy.init_node('image_raw', anonymous=True)
        rate = rospy.Rate(10) # 10hz
        rospy.Subscriber("/Camera_Num", Int16, self.setCamNum)

        # connect to the AirSim simulator 
        self.client = airsim.MultirotorClient()
        self.client.confirmConnection()
        self.client.enableApiControl(True)
        self.camNum = 0

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
                    airsim.ImageRequest("1", airsim.ImageType.Scene, False, False)], vehicle_name="Camera%d" % (self.camNum +1))  #scene vision image in uncompressed RGB array
            # client.simPause(True)
            # # print('Drone1: Retrieved images: %d' % len(responses1))
            # responses2 = client.simGetImages([
            #     airsim.ImageRequest("0", airsim.ImageType.DepthVis),  #depth visualization image
            #     airsim.ImageRequest("1", airsim.ImageType.Scene, False, False)], vehicle_name="Camera1")  #scene vision image in uncompressed RGB array
            # # print('Drone2: Retrieved images: %d' % len(responses2))
            # responses3 = client.simGetImages([
            #     airsim.ImageRequest("0", airsim.ImageType.DepthVis),  #depth visualization image
            #     airsim.ImageRequest("1", airsim.ImageType.Scene, False, False)], vehicle_name="Camera2")  #scene vision image in uncompressed RGB array

            # responses4 = client.simGetImages([
            #     airsim.ImageRequest("0", airsim.ImageType.DepthVis),  #depth visualization image
            #     airsim.ImageRequest("1", airsim.ImageType.Scene, False, False)], vehicle_name="Camera3")  #scene vision image in uncompressed RGB array

            # responses5 = client.simGetImages([
            #     airsim.ImageRequest("0", airsim.ImageType.DepthVis),  #depth visualization image
            #     airsim.ImageRequest("1", airsim.ImageType.Scene, False, False)], vehicle_name="Camera4")  #scene vision image in uncompressed RGB array

            for response1 in responses1:
                img_rgba_string = response1.image_data_uint8
            # for response2 in responses2:
            #     img_rgba_string2 = response2.image_data_uint8
            # for response3 in responses3:
            #     img_rgba_string3 = response3.image_data_uint8
            # for response4 in responses4:
            #     img_rgba_string4 = response4.image_data_uint8
            # for response5 in responses5:
            #     img_rgba_string5 = response5.image_data_uint8

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
            # msg.data = img_rgba_string2
            # pub2.publish(msg)
            # msg.data = img_rgba_string3
            # pub3.publish(msg)
            # msg.data = img_rgba_string4
            # pub4.publish(msg)
            # msg.data = img_rgba_string5
            # pub5.publish(msg)
            # sleep until next cycle
            rate.sleep()

    def setCamNum(self, num):
        if self.camNum is not 4:
            print("Disabling Camera %d" %(self.camNum + 1))
            # self.client.enableApiControl(False, "Camera%d" % (self.camNum +1))
            # print("Disabling Camera %d" %(self.camNum + 1))
            self.camNum = num.data
            # self.client.enableApiControl(True, "Camera%d" % (self.camNum +1))
            print("Enabling Camera %d" %(self.camNum + 1))
            return
        self.camNum = num.data
        print(self.camNum + 1)


if __name__ == '__main__':
    try:
        airpub()
    except rospy.ROSInterruptException:
        pass
