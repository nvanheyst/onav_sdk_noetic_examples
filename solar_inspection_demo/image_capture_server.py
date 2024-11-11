#! /usr/bin/env python3

import actionlib
from clearpath_navigation_msgs.msg import (
    UITaskAction,
    UITaskGoal,
    UITaskActionGoal,
    UITaskActionResult,
    UITaskActionFeedback,
)
import rospy
import math
import cv2 as cv
import numpy as np
import requests
from datetime import datetime
from nav_msgs.msg import Odometry
from onav_sdk.onav_api import *

FLIR_URL = "http://192.168.131.11/api/image/current?imgformat=JPEG"
AXIS_HOST = "192.168.131.10"
AXIS_SNAPSHOT_API_URL = "http://" + AXIS_HOST + "/axis-cgi/jpg/image.cgi"
ODOMETRY_TOPIC = "/husky_velocity_controller/odom"
LOGGING_PREFIX = "custom_task_adaptor"
DEFAULT_DISTANCE_THRESHOLD = 1.0
CAPTURE_SERVER_NAME = "/capture_server"
#SAVE_IMAGE_DIRECTORY= "/opt/onav/0.14.0/app/docker/onav_sdk/scripts/nathan/solar_app/pictures"
SAVE_IMAGE_DIRECTORY= "/workspace/src/onav_sdk/scripts/nathan/solar_app/pictures"

class CustomTaskAdaptor:
    def __init__(self):
        self.distance_travelled_ = 0.0
        self.last_odom_msg_ = None
        self.odom_subscriber_ = rospy.Subscriber(ODOMETRY_TOPIC, Odometry, self.odomCallback)
        self.custom_task_as_ = actionlib.SimpleActionServer(
            CAPTURE_SERVER_NAME,
            UITaskAction,
            execute_cb=self.execute_cb,
            auto_start=False,
        )
        self.custom_task_as_.start()
        self.distance_threshold_ = DEFAULT_DISTANCE_THRESHOLD
        self.enable_capture_ = False
        rospy.loginfo("Server started")

    def captureSnapshot(self):
        resp = requests.get(AXIS_SNAPSHOT_API_URL, stream=True).raw
        image = np.array(bytearray(resp.read()), dtype="uint8")
        image = cv.imdecode(image, cv.IMREAD_COLOR)
        file_name = str(datetime.now())
        img_path = f"{SAVE_IMAGE_DIRECTORY}/{file_name}_PTZ.jpg"
        cv.imwrite(filename=img_path, img=image)
        rospy.loginfo("Saved PTZ image")
        
    def captureThermalImg(self):
        response = requests.get(FLIR_URL)
        file_name = str(datetime.now()) + "_thermal.jpg"
        img_path = SAVE_IMAGE_DIRECTORY + file_name
        rospy.loginfo(f"   Saving thermal file {img_path}")
        with open(img_path, 'wb') as f:
            f.write(response.content)
        rospy.loginfo("Saved Thermal image")

    def odomCallback(self, msg):
            if self.last_odom_msg_ is None:
                self.last_odom_msg_ = msg

            dx = msg.pose.pose.position.x - self.last_odom_msg_.pose.pose.position.x
            dy = msg.pose.pose.position.y - self.last_odom_msg_.pose.pose.position.y
            self.distance_travelled_ += math.sqrt(dx**2+dy**2)
            self.last_odom_msg_ = msg

    def execute_cb(self, goal):
        rospy.loginfo(f"Received goal: {goal}")
        if len(goal.floats) == 0:
            self.distance_threshold_ = DEFAULT_DISTANCE_THRESHOLD
            rospy.loginfo(f"Using default distance " + str(DEFAULT_DISTANCE_THRESHOLD))
        else:
            self.distance_threshold_ = goal.floats[0]
            rospy.loginfo(f"Using default distance " + str(goal.floats[0]))
        if goal.strings[0] == "start":
            rospy.loginfo(f"Starting capture (every {self.distance_threshold_} meters)")
            self.enable_capture_ = True
        else:
            rospy.loginfo(f"Stopping capture")
            self.enable_capture_ = False

        result = UITaskActionResult()
        # result.result.success = True
        # self.custom_task_as_.set_succeeded(result)

    def run(self): 
        while not rospy.is_shutdown():
            if self.enable_capture_:
                if (self.distance_travelled_ > self.distance_threshold_):
                    self.captureSnapshot()
                    #self.captureThermalImg()
                    self.distance_travelled_ = 0

if __name__ == "__main__":
    rospy.init_node("image_capture_server")
    task_adaptor = CustomTaskAdaptor()
    task_adaptor.run()

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
