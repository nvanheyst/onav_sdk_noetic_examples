#! /usr/bin/env python3


from onav_sdk.onav_api import *
import rospy
import math

#SLEEP = 2
DURATION = 48

def on_shutdown():
    rospy.logwarn("Shutdown signal received")
    reset(0)
    rospy.sleep(2)
    stop()
    rospy.sleep(3)

def send_velocity(vel):
    req = ONAV_PTZ_MOVE_VELOCITY.GoalType()
    req.pan = vel
    ONAV_PTZ_MOVE_VELOCITY.send_goal(req)

def reset(pos):
    goal = ONAV_PTZ_MOVE_ABSOLUTE.GoalType()
    goal.pan = pos
    goal.tilt = 0
    goal.zoom = 0
    ONAV_PTZ_MOVE_ABSOLUTE.send_goal_and_wait(goal)

def stop():
    req = ONAV_PTZ_MOVE_RELATIVE.GoalType()
    req.pan = 0
    req.tilt = 0
    rospy.loginfo("Stopping")
    ONAV_PTZ_MOVE_RELATIVE.send_goal_and_wait(req)
   

def main():

    rospy.init_node("optional_pan")
    
    while not rospy.is_shutdown():
        reset(-math.pi/2)
        rospy.sleep(2)
        stop()
        send_velocity(0.1)
        rospy.loginfo("Moving clockwise 1 rad at 0.1 rad/s")
        rospy.sleep(DURATION)
        reset(math.pi/2)
        rospy.sleep(2)
        stop()
        send_velocity(-0.1)
        rospy.loginfo("Moving couter clockwise clockwise 1 rad at 0.1 rad/s")
        rospy.sleep(DURATION)

    stop()

main()

