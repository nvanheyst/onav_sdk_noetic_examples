# patrol_main

This is patrol example. It was mainly created to as an example and to test the onav_sdk, but could serve as a useful template and starting point for a security application with the Husky Observer.

Here is an outline of the features of the program:
-

**Options**

Inference
Currently the program uses a roboflow inference server setup on an Orin NX: https://inference.roboflow.com/quickstart/docker/
Alternatives are shown in the options folder such as using roboflows cloud server or running an object detection model on the robot

Pan
The program is setup by default to run with just the camera pointing forward, but the pan_velocity_continuous.py node can be run in parallel to expand the FOV
