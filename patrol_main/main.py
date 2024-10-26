#! /usr/bin/env python3

from datetime import datetime

import cv2
from cv_bridge import CvBridge, CvBridgeError
from inference_sdk import InferenceHTTPClient
import rospy
import supervision as sv

from onav_sdk.onav_api import *


DETECTION_MODEL = "coco/24" # Yolo arch running on coco trained model
DETECTION_ENDPOINT = "http://10.29.10.28:9001" # IP for Stereolabs detection server
POI_TAG_NAME = "patrol.x"
MAP_UUID = "c85ee84f-3af5-46d9-935c-b54c1939f159"  # Greenspace
DOCK_UUID = "09669899-3f70-4943-981e-1df4c15175eb"  # Dock 1
FLASH_LIGHTS_INTERVAL = 0.4 # Toggle lights on/off every 0.4 seconds
INTRUDER_ALARM_DURATION = 10 # How long the intruder routine runs for
SAVE_IMAGE_FOLDER = "/opt/onav/saved_files/media/axis_q62" # Annotated intruder images saved here
CHARGE_THRESHOLD_LOW = 0.1
CHARGE_THRESHOLD_HIGH = 0.6
NUM_PATROL_LOOPS = 100
INTRUDER_LOOK_RATE = 1 # Analyze images every 1 second to look for people


class ObserverIntruderNode:
    def __init__(self):
        self.detection_client_ = InferenceHTTPClient(
            api_url=DETECTION_ENDPOINT
        )
        self.cv_bridge_ = CvBridge()
        self.bounding_box_annotator_ = sv.BoundingBoxAnnotator()

    def drawDetectionsOnImage(self, image, result):
        detections = sv.Detections.from_inference(result)
        annotated_image = self.bounding_box_annotator_.annotate(
            scene=image, detections=detections
        )
        return annotated_image

    def runDetector(self, image):
        try:
            result = self.detection_client_.infer(image, model_id=DETECTION_MODEL)
            return result
        except:
            rospy.logerr(f"Caught exception while performing inference, resting for 10s")
            rospy.sleep(10)
            return None

    def startLogging(self, name):
        start_req = ONAV_LOGGER_START_RECORDING.RequestType()
        start_req.name = name
        ONAV_LOGGER_START_RECORDING.call(start_req)
        rospy.sleep(2)

    def stopLogging(self):
        rospy.sleep(2)
        stop_request = ONAV_LOGGER_STOP_RECORDING.RequestType()
        ONAV_LOGGER_STOP_RECORDING.call(stop_request)

    def getPOIS(self):
        response = ONAV_MM_GET_ALL_POIS.call()
        pois = []
        for i in response.points_of_interest:
            if POI_TAG_NAME in i.tags:
                pois.append(i)
        return pois

    def sendPOIGoal(self, uuid):
        goal = ONAV_AUTONOMY_GOTO_POI.GoalType()
        goal.poi_uuid = uuid
        goal.map_uuid = MAP_UUID
        ONAV_AUTONOMY_GOTO_POI.send_goal_and_wait(goal)

    def intruderAlert(self):
        t_start = rospy.get_time()

        rospy.logwarn("Intruder detected! Flashing lights and sounding alarm!")
        ONAV_PLATFORM_SET_SOUND_ON.set_true()
        ONAV_PLATFORM_SET_STACK_LIGHT_STROBE.set_true()
        rospy.sleep(0.02)
        while rospy.get_time() - t_start < INTRUDER_ALARM_DURATION:
            ONAV_PLATFORM_SET_FRONT_LIGHTS_ON.set_false()
            rospy.sleep(0.1)
            ONAV_PLATFORM_SET_REAR_LIGHTS_ON.set_false()
            rospy.sleep(0.1)
            ONAV_PLATFORM_SET_SOUND_ON.set_true()
            rospy.sleep(0.1)
            rospy.sleep(FLASH_LIGHTS_INTERVAL)
            ONAV_PLATFORM_SET_FRONT_LIGHTS_ON.set_true()
            rospy.sleep(0.1)
            ONAV_PLATFORM_SET_REAR_LIGHTS_ON.set_true()
            rospy.sleep(0.1)
            ONAV_PLATFORM_SET_SOUND_ON.set_false()
            rospy.sleep(FLASH_LIGHTS_INTERVAL)

        ONAV_PLATFORM_SET_STACK_LIGHT_STROBE.set_false()
        rospy.sleep(0.1)

        ONAV_PLATFORM_SET_SOUND_ON.set_false()
        rospy.sleep(0.1)

        ONAV_PLATFORM_SET_FRONT_LIGHTS_ON.set_false()
        rospy.sleep(0.1)

        ONAV_PLATFORM_SET_REAR_LIGHTS_ON.set_false()
        rospy.sleep(0.1)

    def lookForIntruders(self):
        img_msg = ONAV_TOPIC_CAMERA_0_IMAGE.latest()
        cv_img = self.cv_bridge_.compressed_imgmsg_to_cv2(
            img_msg, desired_encoding="passthrough"
        )

        detection_results = self.runDetector(cv_img)
        if detection_results is None:
            return
            
        if len(detection_results["predictions"]) > 0:
            confident = False
            for p in detection_results["predictions"]:
                if p["confidence"] > 0.75 and p["class"] == "person":
                    confident = True
                    rospy.loginfo(f"Detection: {detection_results}")
                    break
            if not confident:
                return

            # Pause autonomy
            ONAV_AUTONOMY_PAUSE.set_true()

            # Run alarm routine
            self.intruderAlert()

            # Save an annotated image of the intruder
            img = self.drawDetectionsOnImage(cv_img, detection_results)
            fname = round(rospy.get_time())
            detected_file_path = f"{SAVE_IMAGE_FOLDER}/{fname}_detection.png"
            cv2.imwrite(filename=detected_file_path, img=img)

            # Register the annotated image with logger so it shows up in log
            req = ONAV_LOGGER_RECORD_MEDIA_EVENT.RequestType()
            req.media_path = detected_file_path
            req.mime_type = "image/png"
            ONAV_LOGGER_RECORD_MEDIA_EVENT.call(req)

            # Not necessary, but we could also log the intrusion as an error event (shows up in UI)
            req = ONAV_LOGGER_RECORD_ERROR.RequestType()
            req.error_msg = "Person detected"
            ONAV_LOGGER_RECORD_ERROR.call(req)

            # Resume autonomy
            ONAV_AUTONOMY_RESUME.set_true()

    def chargeRobot(self):
        dock_goal = ONAV_AUTONOMY_DOCK_NETWORK.GoalType()
        dock_goal.network_uuid = "c85ee84f-3af5-46d9-935c-b54c1939f159"
        dock_goal.dock_uuid = "09669899-3f70-4943-981e-1df4c15175eb"
        ONAV_AUTONOMY_DOCK_NETWORK.send_goal_and_wait(dock_goal)

        r = rospy.Rate(10)
        charge = 0.0
        while charge < CHARGE_THRESHOLD_HIGH:
            msg = ONAV_TOPIC_BATTERY_STATE_WITH_LVC.latest()
            charge = msg.percentage
            r.sleep()

        rospy.loginfo("Finished charging, undocking and resuming patrol")
        undock_goal = ONAV_AUTONOMY_UNDOCK.GoalType()
        undock_goal.undock_distance = 2
        ONAV_AUTONOMY_UNDOCK.send_goal_and_wait(undock_goal)

    def onShutdown(self):
        rospy.logwarn("Shutdown signal received - stopping autonomy")
        ONAV_AUTONOMY_STOP.call()
        rospy.logwarn("Stopping logger")
        self.stopLogging()
        rospy.logwarn("Shutdown")

    def run(self):
        rospy.init_node("onav_patrol_application")
        rospy.on_shutdown(self.onShutdown)

        self.startLogging(f"Better Patrol Mission - {datetime.now()}")

        # Undock the robot
        undock_goal = ONAV_AUTONOMY_UNDOCK.GoalType()
        undock_goal.undock_distance = 2
        result = ONAV_AUTONOMY_UNDOCK.send_goal_and_wait(undock_goal)

        # Start the patrol
        look_rate = rospy.Rate(INTRUDER_LOOK_RATE)
        pois = self.getPOIS()
        for i in range(NUM_PATROL_LOOPS):
            for poi in pois:
                rospy.loginfo(f"Loop {i} - Navigating to POI {poi.name}")
                msg = ONAV_TOPIC_BATTERY_STATE_WITH_LVC.latest()
                if msg.percentage < CHARGE_THRESHOLD_LOW:
                    rospy.loginfo(f"Robot charge is {msg.percentage}, returning to dock")
                    self.chargeRobot()
                goal = ONAV_AUTONOMY_GOTO_POI.GoalType()
                goal.poi_uuid = poi.uuid
                goal.map_uuid = MAP_UUID
                ONAV_AUTONOMY_GOTO_POI.send_goal(goal)
                while (
                    not ONAV_AUTONOMY_GOTO_POI.has_result() and not rospy.is_shutdown()
                ):
                    self.lookForIntruders()
                    look_rate.sleep()
                if rospy.is_shutdown():
                    rospy.logwarn("is_shutdown true")
                    return

        self.stopLogging()


if __name__ == "__main__":
    patrol_node = ObserverIntruderNode()
    patrol_node.run()
