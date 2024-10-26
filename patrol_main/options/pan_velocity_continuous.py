#! /usr/bin/env python3

import math
import random
import rospy
from onav_sdk.onav_api import *
from std_msgs.msg import Bool



length = 16
s = 3

number = random.randrange(1000,100000000)


def main():
    rospy.init_node("security_pan")

    

    while not rospy.is_shutdown():

        reset(-math.pi/2)

        rospy.sleep(s)

        pan(0.1)

        for i in range(length):
            rospy.sleep(s)

            rospy.loginfo(str((i+1)*s) + " seconds")

            #save_image()

        cancel()

        rospy.sleep(s)
        
        reset(math.pi/2)

        rospy.sleep(s)

        pan(-0.1)

        for i in range(length):
            rospy.sleep(s)

            rospy.loginfo(str((i+1)*s) + " seconds")

        cancel()

        rospy.sleep(s)
        

    

def on_shutdown():
    rospy.logwarn("Shutdown signal received")
    cancel()
    rospy.sleep(s)
    reset(0)
    rospy.sleep(s)


def reset(pos):
    goal = ONAV_PTZ_MOVE_ABSOLUTE.GoalType()
    goal.pan = pos
    goal.tilt = 0
    goal.zoom = 0
    ONAV_PTZ_MOVE_ABSOLUTE.send_goal_and_wait(goal)

def cancel():
    ONAV_PTZ_MOVE_VELOCITY.cancel()

def save_image():
    global number
    goal = ONAV_PLATFORM_SAVE_IMAGE_CAMERA_0.GoalType()
    goal.filename = str(number)
    ONAV_PLATFORM_SAVE_IMAGE_CAMERA_0.send_goal(goal)
    number +=1
    rospy.loginfo("saved image " + str(number))

def pan(vel):
    goal = ONAV_PTZ_MOVE_VELOCITY.GoalType()
    goal.pan = vel
    ONAV_PTZ_MOVE_VELOCITY.send_goal(goal)

if __name__ == "__main__":
    main()
