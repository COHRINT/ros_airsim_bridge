#!/usr/bin/env python

# Example ROS node for publishing AirSim images.

# AirSim Python API
import setup_path 
import airsim
import sys, select, termios, tty, signal, time

import rospy

# ROS Image message
from sensor_msgs.msg import Image
from std_msgs.msg import Int16
from geometry_msgs.msg import PoseStamped


class airpub():
    def __init__(self):
        # self.image_pub = rospy.Publisher("Drone1/image_raw", Image, queue_size=1)
        # self.state_pub = rospy.Publisher("Drone1/pose", PoseStamped, queue_size=1)
        # rospy.Subscriber("/Camera_Num", Int16, self.setCamNum)
        # rospy.Subscriber("/Drone1/Goal", PoseStamped, self.moveToGoal)
        # rospy.init_node('image_publisher', anonymous=False)
        # self.rate = rospy.Rate(10) # 10hz
        # rospy.Subscriber("/Camera_Num", Int16, self.setCamNum)

        self.camCoords = [[100, 100, -2], [3900, 100, -2], [100, 3900, -2], [3900, 39   00, -5]]

        # connect to the AirSim simulator 
        self.client = airsim.MultirotorClient()
        self.client.confirmConnection()
        self.client.enableApiControl(True, "Camera1")
        self.client.enableApiControl(True, "Drone1")
        # self.client.armDisarm(True, "Drone1")
        # self.client.armDisarm(True, "Camera1")
        # self.client.takeoffAsync(vehicle_name="Drone1").join()
        self.camNum = 0
        self.speed = 10
        self.altitude = 20

        self.pose = self.client.simGetVehiclePose()



    
    def start(self):

        self.image_pub = rospy.Publisher("Drone1/image_raw", Image, queue_size=1)
        self.state_pub = rospy.Publisher("Drone1/pose", PoseStamped, queue_size=1)
        rospy.Subscriber("/Camera_Num", Int16, self.setCamNum)
        # rospy.Subscriber("/Drone1/Goal", PoseStamped, self.moveToGoal)
        rospy.init_node('image_publisher', anonymous=False)
        self.rate = rospy.Rate(10)

        print("Arming drone...")
        self.client.armDisarm(True, vehicle_name="Drone1")
        self.client.armDisarm(True, vehicle_name="Camera1")

        # landed = self.client.getMultirotorState(vehicle_name="Drone1").landed_state
        # if landed == airsim.LandedState.Landed:
        #     print("taking off...")
        #     self.client.takeoffAsync(vehicle_name="Drone1").join()

        # landed = self.client.getMultirotorState(vehicle_name="Drone1").landed_state
        # if landed == airsim.LandedState.Landed:
        #     print("takeoff failed - check Unreal message log for details")
        #     return

        # print(self.client.isApiControlEnabled(vehicle_name = 'Drone1'))

        # # self.client.confirmConnection()

        # z = -self.altitude

        # print("climbing to altitude: " + str(self.altitude))
        # self.client.moveToPositionAsync(0, 0, z, self.speed, vehicle_name="Drone1").join()

        # # let it settle there a bit.
        # self.client.hoverAsync().join()
        # time.sleep(2)

        # # after hovering we need to re-enabled api control for next leg of the trip
        # self.client.enableApiControl(True)

        # print("Altitude reached")

        while not rospy.is_shutdown():

            self.publishState()
            # self.drone_state = self.client.getMultirotorState(vehicle_name="Drone1")

            # get camera images from the car
            print(self.camNum)
            # client.simPause(False)
            if self.camNum is 4:
                # print("Drone")
                # responses1 = client.simGetImage("0", airsim.ImageType.Scene, "Drone1")
                # img_rgba_string = responses1
                responses1 = self.client.simGetImages([
                    airsim.ImageRequest("0", airsim.ImageType.DepthVis),  #depth visualization image
                    airsim.ImageRequest("1", airsim.ImageType.Scene, False, False)], vehicle_name="Drone1")  #scene vision image in uncompressed RGB array
            else:
                # print("Camera")
                responses1 = self.client.simGetImages([
                    airsim.ImageRequest("0", airsim.ImageType.DepthVis),  #depth visualization image
                    airsim.ImageRequest("1", airsim.ImageType.Scene, False, False)], vehicle_name="Camera1")  #scene vision image in uncompressed RGB array
         
            for response1 in responses1:
                img_rgba_string = response1.image_data_uint8

            # Populate image message
            msg=Image() 
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = "frameId"
            msg.encoding = "bgr8"
            msg.height = 240  # resolution should match values in settings.json 
            msg.width = 480
            msg.data = img_rgba_string
            msg.is_bigendian = 0
            msg.step = msg.width * 3


            # log time and size of published image
            # rospy.loginfo(len(response1.image_data_uint8))
            # publish image message
            self.image_pub.publish(msg)
            self.rate.sleep()
        
        self.client.armDisarm(False)

    def moveToGoal(self, data):
 
        print("Moving to goal")

        # drone_state = self.client.getMultirotorState(vehicle_name="Drone1")
        # # print(drone_state)
        # # drone_state = self.client.simGetVehiclePose()
        # pos_ned = self.drone_state.kinematics_estimated.position

        # self.client.moveToPositionAsync(self.pose.position.x_val, self.pose.position.y_val, self.pose.position.z_val, self.speed, timeout_sec=10, drivetrain=airsim.DrivetrainType.ForwardOnly, yaw_mode=airsim.YawMode(is_rate=False, yaw_or_rate=0)).join()
        # self.client.moveToPositionAsync(data.pose.position.x, data.pose.position.y, -self.altitude, self.speed, timeout_sec=10, drivetrain=airsim.DrivetrainType.ForwardOnly, yaw_mode=airsim.YawMode(is_rate=False, yaw_or_rate=0), vehicle_name="Drone1").join()


    def setCamNum(self, num):
        # self.client.confirmConnection()
        # self.client.enableApiControl(True, "Camera1")
        # self.client.armDisarm(True, "Camera1")
        print("CamNum Callback:")

        self.camNum = num.data
        
        if self.camNum is not 4:


            self.pose.position.x_val = self.camCoords[self.camNum][0]
            self.pose.position.y_val = self.camCoords[self.camNum][1]
            self.pose.position.z_val = self.camCoords[self.camNum][2]
            # self.pose.orientation.

            self.client.simSetVehiclePose(self.pose, True, vehicle_name="Camera1").join()

    def publishState(self):
        drone_state = self.client.getMultirotorState(vehicle_name="Drone1")
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
        return
        


if __name__ == '__main__':
    try:
        airpub = airpub()
        airpub.start()
    except rospy.ROSInterruptException:
        pass
