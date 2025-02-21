# solar_inspection_demo

This is an inspection POC for the Clearpath Robotics Husky Observer. This is not intended for commercial deployment and is just an example, but could serve as a starting point for additional development.

The purpose of this example is to create automated missions for the purpose of underpanel solar inspections such as in this video: https://www.youtube.com/watch?v=xIM0MXvQyGU. This program assumes equidistant spacing between panel rows, even terrain and limited obstacles.

<ins>Here is an outline of the features of the program: </ins>
- takes the generated map from the row generator demo as an input
- adds waypoints at the beginning and end of each row
   * exludes the outer edges (program can be modified to include or just add extra rows)
   * starts from the row closest to and in the same direction as the first edge in the generator polygon
   * calculates heading automatically (counter can be started at counter=3 to reverse the direction of travel)
   * can be extended to add interpolated points on each row (example can be provided)
- adds an automated PTZ camera rotation to the beginning waypoint of each row (the values of pan1 and pan2 can be interchanged to get the opposite effect
- option to add video capture or automated image capture. image_capture_server.py shows a basic example of automated image capture for a specified distance interval. It is setup as a custom action server for OutdoorNav. This can be set as an "on start task" and take picture during the turns or it can be programmatically started and stopped at the beginning and end of each row. 

For more information on the Husky Observer: https://clearpathrobotics.com/husky-observer/


![20240503_133329](https://github.com/user-attachments/assets/c708d429-078d-4188-bbf5-738d2e7c2ed0)
![image (68)](https://github.com/user-attachments/assets/b59e637b-f0bf-4acb-b7fb-fecf3c7ad4d7)
![image (69)](https://github.com/user-attachments/assets/4e8fed6d-ca39-4be9-97f7-429905418a64)
