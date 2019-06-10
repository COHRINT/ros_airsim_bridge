#!/usr/bin/env python


import sys, select, termios, tty
import math
import signal

import rospy
from geometry_msgs.msg import Twist

import pprint

import airsim_objects

import os
import threading
import numpy as np
import rospy
import std_msgs.msg
import geometry_msgs.msg
import cv_bridge
import cv2 as opencv
import unrealcv
# import unreal_coordinates as ue_coords
from airsim_bridge.srv import *

# CHANGE PORT AT:
# /home/tetsuo/ws/sim/UnrealEngine/Engine/Binaries/Linux/unrealcv.ini

def read_png(res):
    import StringIO, PIL.Image
    img = PIL.Image.open(StringIO.StringIO(res))
    return np.asarray(img)

class UnrealCVServer():
    """
    A ROS node for the unrealcv API.
    The goal of this node is to exactly mirror the UnrealCV API, documented here:
    http://unrealcv.org/reference/commands.html
    This is based on the source code, at time of writing, the documentation above is incomplete.
    """

    # These are the valid view modes for the cameras.
    view_modes = ['lit', 'depth', 'normal', 'object_mask', 'wireframe']

    def __init__(self, config):
        # host = unrealcv.HOST
        # port = unrealcv.PORT
        #
        # if 'endpoint' in config:
        #     host, port = config['endpoint']
        # if 'port' in config:
        #     port = config['port'] 
        # if 'hostname' in config:
        #     host = config['hostname']

        host = '127.0.0.1'
        port = 9234

        self.opencv_bridge = cv_bridge.CvBridge()

        self._client_lock = threading.Lock()
        self._client = unrealcv.Client(('127.0.0.1', 9234))
        self._client.connect()
        if not self._client.isconnected():
            raise RuntimeError("Could not connect to unrealcv simulator, is it running?")

        res = self._client.request('vget /unrealcv/status')
        print(res)

        # Store the declare services
        self._services = []

    def create_services(self):
        print("Starting services...")
        # Camera control services
        self._services.append(rospy.Service('get_camera_view', GetCameraImage, self.handle_get_camera_image))
        self._services.append(rospy.Service('get_camera_view_with_filename', GetCameraImageWithFilename,
                                            self.handle_get_camera_image))
        self._services.append(rospy.Service('get_camera_location', GetCameraLocation,
                                            self.handle_get_camera_location))
        self._services.append(rospy.Service('get_camera_rotation', GetCameraRotation,
                                            self.handle_get_camera_rotation))
        self._services.append(rospy.Service('get_viewmode', GetViewmode, self.handle_get_viewmode))
        self._services.append(rospy.Service('move_camera', MoveCamera, self.handle_move_camera))
        self._services.append(rospy.Service('set_camera_location', SetCameraLocation,
                                            self.handle_set_camera_location))
        self._services.append(rospy.Service('set_camera_rotation', SetCameraRotation,
                                            self.handle_set_camera_rotation))
        self._services.append(rospy.Service('set_viewmode', SetViewmode, self.handle_set_viewmode))

        # object control services
        # self._services.append(rospy.Service('get_object_color', services.GetObjectColor, self.handle_get_object_color))
        # self._services.append(rospy.Service('set_object_color', services.SetObjectColor, self.handle_set_object_color))
        # self._services.append(rospy.Service('get_object_location', services.GetObjectLocation,
        #                                     self.handle_get_object_location))
        # self._services.append(rospy.Service('get_object_rotation', services.GetObjectRotation,
        #                                     self.handle_get_object_rotation))
        # self._services.append(rospy.Service('set_object_location', services.SetObjectLocation,
        #                                     self.handle_set_object_location))
        # self._services.append(rospy.Service('set_object_rotation', services.SetObjectRotation,
        #                                     self.handle_set_object_rotation))

    def shutdown_services(self, reason=''):
        for service in self._services:
            service.shutdown(reason)
        self._client.disconnect()

    # Helpers and locking
    def request_client(self, request):
        self._client_lock.acquire()
        result = self._client.request(request)
        self._client_lock.release()
        return result

    # Service Handlers
    def handle_get_camera_image(self, request):
        # Parse the request arguments
        # It is also possible to get the png directly without saving to a file
        # res = client.request('vget /camera/0/lit png')
        # im = read_png(res)
        # print(im.shape)
        filename = None
        if hasattr(request, 'filename'):
            filename = request.filename
        view_mode = 'lit'
        if hasattr(request, 'view_mode') and request.view_mode in self.view_modes:
            view_mode = request.view_mode
        # print
        unrealcv_message = make_vget_camera_image(request.camera_id, view_mode, filename)
        image_filename = self.request_client(unrealcv_message)
        print("After Request")
        # print("Image Filename: "+image_filename)
        # if os.path.isfile(image_filename):
        #     image_mat = opencv.imread(image_filename)
        #     # os.remove(image_filename)
        #     print("removing")
        # else:
        #     print("file does not exist")
        #     # image_mat = np.matrix([[]])
        #     image_mat = opencv.imread(image_filename)

        image_mat = read_png(image_filename)
        image_mat = np.array(image_mat)
        image_mat = opencv.cvtColor(image_mat, opencv.COLOR_RGB2BGR)

        return self.opencv_bridge.cv2_to_imgmsg(image_mat, encoding='passthrough')

    def handle_get_camera_location(self, request):
        message = make_vget_camera_location(request.camera_id)
        location = self.request_client(message)
        location, _ = transform_from_unreal(location, None)
        return geometry_msgs.msg.Point(x=location[0], y=location[1], z=location[2])

    def handle_get_camera_rotation(self, request):
        message = make_vget_camera_location(request.camera_id)
        rotation = self.request_client(message)
        _, rotation = transform_from_unreal(None, rotation)
        return geometry_msgs.msg.Quaternion(w=rotation[0], x=rotation[1], y=rotation[2], z=rotation[3])

    def handle_get_viewmode(self, request):
        return self.request_client(make_vget_viewmode())

    def handle_move_camera(self, request):
        message = make_vset_move_camera(request.camera_id, request.location.x, request.location.y, request.location.z)
        return self.request_client(message)

    def handle_set_camera_location(self, request):
        message = make_vset_camera_location(request.camera_id, request.location.x,
                                            request.location.y, request.location.z)
        return self.request_client(message)

    def handle_set_camera_rotation(self, request):
        message = make_vset_camera_rotation(request.camera_id, request.rotation.w, request.rotation.x,
                                            request.rotation.y, request.rotation.z)
        return self.request_client(message)

    def handle_set_viewmode(self, request):
        view_mode = 'lit'
        if hasattr(request, 'view_mode') and request.view_mode in self.view_modes:
            view_mode = request.view_mode
        return self.request_client(make_vset_viewmode(view_mode))


def transform_to_unreal(location, rotation):
    """
    Swap the coordinate frames from the ROS standard coordinate frame to the one used by unreal
    :param location: A point, as any 3-length indexable,
    :param rotation: An orientation as a quaternion, w first
    :return: A tuple containing location and rotation (in euler angles)
    """
    if location is not None and len(location) >= 3:
        location = (location[0], location[1], location[2])
    else:
        location = (0, 0, 0)
    if rotation is not None and len(rotation) >= 4:
        rotation = (rotation[0], rotation[1], rotation[2], rotation[3])
    else:
        rotation = (1, 0, 0, 0)

    # Invert y axis
    location = (location[0], -location[1], location[0])
    rotation = (rotation[0], rotation[1], -rotation[2], rotation[3])

    # Invert rotation for left-handed switch
    rotation = (np.array((1, -1, -1, -1)) * rotation) / np.dot(rotation, rotation)

    return location, unreal_quat2euler(rotation[0], rotation[1], rotation[2], rotation[3])

def unreal_quat2euler(w, x, y, z):
    """
    Convert a quaternion in unreal space to euler angles.
    Based on FQuat::Rotator in
    Engine/Source/Runtime/Core/Private/Math/UnrealMath.cpp ln 536
    which is in turn based on
    https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
    :param w:
    :param x:
    :param y:
    :param z:
    :return:
    """
    SINGULARITY_THRESHOLD = 0.4999995

    singularity_test = z * x - w * y
    yaw_y = 2 * (w * z + x * y)
    yaw_x = 1 - 2 * (y * y + z * z)

    yaw = np.arctan2(yaw_y, yaw_x) * (180 / np.pi)
    if singularity_test < -SINGULARITY_THRESHOLD:
        pitch = -90
        roll = _clamp_axis(-yaw - 2 * np.arctan2(x, w) * (180 / np.pi))
    elif singularity_test > SINGULARITY_THRESHOLD:
        pitch = 90
        roll = _clamp_axis(yaw - 2 * np.arctan2(x, w) * (180 / np.pi))
    else:
        pitch = np.arcsin(2 * singularity_test) * (180 / np.pi)
        roll = np.arctan2(-2 * (w * x + y * z), (1 - 2 * (x * x + y * y))) * (180 / np.pi)

# Helper messages to create UnrealCV message URIs
def make_vget_camera_image(camera_id, view_mode, filename):
    print(str(view_mode))
    print(str(filename))
    if filename is None:
        # return "vget /camera/{0}/{1}".format(camera_id, view_mode)
        print("filename none")
        return "vget /camera/0/lit png"
    else:
        return "vget /camera/{0}/{1} {2}".format(camera_id, view_mode, filename)


def make_vget_camera_location(camera_id):
    return "vget /camera/{0}/location".format(camera_id)


def make_vget_camera_rotation(camera_id):
    return "vget /camera/{0}/rotation".format(camera_id)


def make_vget_viewmode():
    return "vget /viewmode"


def make_vset_move_camera(camera_id, x, y, z):
    location, _ = transform_to_unreal((x, y, z), None)
    return "vset /camera/{0}/moveto {1} {2} {3}".format(camera_id, location[0], location[1], location[2])


def make_vset_camera_location(camera_id, x, y, z):
    location, _ = transform_to_unreal((x, y, z), None)
    return "vset /camera/{0}/location {1} {2} {3}".format(camera_id, location[0], location[1], location[2])


def make_vset_camera_rotation(camera_id, w, x, y, z):
    _, rotation = transform_to_unreal(None, (w, x, y, z))
    roll, pitch, yaw = rotation
    return "vset /camera/{0}/rotation {1} {2} {3}".format(camera_id, pitch, yaw, roll)


def make_vset_viewmode(viewmode):
    return "vset /viewmode {0}".format(viewmode)

if __name__ == '__main__':
    try:
        # unrealCVServer()
        rospy.init_node('unrealcv_server')
        unrealcv_bridge = UnrealCVServer(config={})  # TODO: Get config from somewhere
        unrealcv_bridge.create_services()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
