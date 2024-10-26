# patrol_robot

This is patrol POC for the Clearpath Robotics Husky Observer. This is not intended for commercial deployment and is just an example, but could serve as a template for additional development.

Here is an outline of the features of the program:
- cycle between specificied POIs
- log data and events to execution logger
- return to dock is SOC gets below threshold and restart when charged
- check image stream on the PTZ for human detections
- save an annotated image of detections
- flash lights and sound alarm at detected intruders

![image (53)](https://github.com/user-attachments/assets/54160a60-9311-4bc2-8ef1-fdf4700cff1b)
![20240417_212424](https://github.com/user-attachments/assets/4896be0f-7c0f-4290-8f7c-ff3793763020)

**Options**

Inference
Currently the program uses a Roboflow inference server setup on an Orin NX: https://inference.roboflow.com/quickstart/docker/
Alternatives can be used such as using Roboflow's cloud server or running an object detection model on the robot

Pan
The program is setup by default to run with just the camera pointing forward, but the pan_velocity_continuous.py node can be run in parallel to expand the FOV by panning back and forth


