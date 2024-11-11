# patrol_robot

This is a patrol application POC for the Clearpath Robotics Husky Observer. This is not intended for commercial deployment and is just an example, but could serve as a starting point for additional development.

The purpose of this program was to show how to use POIs (points of interest) within an application, show a security example and to show how to integrate computer vision model inference.

<ins>Here is an outline of the features of the program: </ins>
- cycle between specified POIs (GPS coordinates specified as patrol points)
- check image stream on the PTZ for human detections
- save an annotated image of detections
- flash lights and sound alarm at detected intruders
- log data and events to execution logger
- return to dock is SOC gets below threshold and restart when charged

For more information on the Husky Observer: https://clearpathrobotics.com/husky-observer/

![image (53)](https://github.com/user-attachments/assets/54160a60-9311-4bc2-8ef1-fdf4700cff1b)
![image](https://github.com/user-attachments/assets/644171b3-4f55-43d4-92b6-02af74f29f26)

<ins> **Options** </ins>

**Inference**

Currently the program uses a Roboflow inference server setup on an Orin NX: https://inference.roboflow.com/quickstart/docker/
Alternatives can be used such as using Roboflow's cloud server or running an object detection model on the robot

**Pan**

The program is setup by default to run with just the camera pointing forward, but their is also an optional_pan node can be run in parallel to expand the FOV by panning back and forth


