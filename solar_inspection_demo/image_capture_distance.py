#!/usr/bin/env python3

import actionlib
from clearpath_navigation_msgs.msg import UITaskAction, UITaskGoal, UITaskActionGoal, UITaskActionResult, UITaskActionFeedback, UITaskResult
from nav_msgs.msg import Odometry
import rospy

from datetime import datetime
import cv2 as cv
import numpy as np
import requests
import math

FLIR_URL = "http://192.168.131.11/api/image/current?imgformat=JPEG"
AXIS_HOST = "192.168.131.10"
AXIS_SNAPSHOT_API_URL = "http://" + AXIS_HOST + "/axis-cgi/jpg/image.cgi"
ODOMETRY_TOPIC = "/husky_velocity_controller/odom"
DEFAULT_DISTANCE_THRESHOLD = 1.0
CAPTURE_SERVER_NAME = "/capture_server"
SAVE_IMAGE_DIRECTORY= "/home/administrator/nathan_ws/pictures/"

class TakePicturesWhileDrivingTask:
    def __init__(self):
        self.distance_travelled_ = 0.0
        self.last_odom_msg_ = None
        self.odom_subscriber_ = rospy.Subscriber(ODOMETRY_TOPIC, Odometry, self.odomCallback)
        self.capture_server_ = actionlib.SimpleActionServer(CAPTURE_SERVER_NAME, UITaskAction, execute_cb=self.startCaptureCallback, auto_start=False)
        self.capture_server_.start()
        self.distance_threshold_ = DEFAULT_DISTANCE_THRESHOLD
        self.enable_capture_ = False


    def captureSnapshot(self):
        resp = requests.get(AXIS_SNAPSHOT_API_URL, stream=True).raw
        image = np.asarray(bytearray(resp.read()), dtype="uint8")
        image = cv.imdecode(image, cv.IMREAD_COLOR)
        return image

    def captureThermalImg(self):
        response = requests.get(FLIR_URL)
        file_name = str(datetime.now()) + "_thermal.jpg"
        img_path = SAVE_IMAGE_DIRECTORY + file_name
        rospy.loginfo(f"   Saving thermal file {img_path}")
        with open(img_path, 'wb') as f:
            f.write(response.content)

    def startCaptureCallback(self, goal):
        if len(goal.floats) == 0:
            self.distance_threshold_ = DEFAULT_DISTANCE_THRESHOLD
        else:
            self.distance_threshold_ = goal.floats[0]


        if goal.strings[0] == "start":
            rospy.loginfo(f"Starting capture (every {self.distance_threshold_} meters)")
            self.enable_capture_ = True
        else:
            rospy.loginfo(f"Stopping capture")
            self.enable_capture_ = False

        result = UITaskResult()
        result.success = True
        self.capture_server_.set_succeeded(result)
        
    def odomCallback(self, msg):
        if self.last_odom_msg_ is None:
            self.last_odom_msg_ = msg

        dx = msg.pose.pose.position.x - self.last_odom_msg_.pose.pose.position.x
        dy = msg.pose.pose.position.y - self.last_odom_msg_.pose.pose.position.y
        self.distance_travelled_ += math.sqrt(dx**2+dy**2)
        self.last_odom_msg_ = msg


    def run(self): 
        while not rospy.is_shutdown():
            if self.enable_capture_:
                if (self.distance_travelled_ > self.distance_threshold_):
                    self.captureThermalImg()
                    file_name = str(datetime.now()) + ".jpg"
                    img = self.captureSnapshot()
                    img_path = SAVE_IMAGE_DIRECTORY + file_name
                    rospy.loginfo(f"   Saving file {img_path}")
                    cv.imwrite(img_path, img)
                    self.distance_travelled_ = 0
        
if __name__ == '__main__':
    rospy.init_node('pictures_over_distance')
    
    obj = TakePicturesWhileDrivingTask()
    obj.run()
