### onav_sdk_examples

**Background**

OutdoorNav is a ROS based navigation stack primarily designed for deployment on Clearpath robots. It uses GPS for localization and 3D LiDAR for obstacled detection.

The onav_sdk is a set of light-wrappers around ROS services, actions, and topics described in the OutdoorNav ROS API. 

This repo is to share work in progress examples. They aren't overly polished and shouldn't be considered viable for commercial deployment as is. Instead they should be considered as examples and could serve as a useful starting point for additional application development. 

**For current examples with ROS2 and the Husky A300/AMP please see the ROS2 repo: https://github.com/nvanheyst/ros2-outdoornav-api-examples**


**Get Access to the onav_sdk**

If you have OutdoorNav 0.13.0 or newer - good news! You already have access to the onav_sdk. 

The VS Code Server is only accessible via - http://(hostname)/_/code/

If the link doesn't work you might need to start the docker. Here are the two steps required (change 0.14.1 to the version you are running):

$cd /opt/onav/0.14.1/app 
$docker compose up -d code

Some examples are provided in the scripts folder.
