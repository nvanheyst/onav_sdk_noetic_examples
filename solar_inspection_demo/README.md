# patrol_robot

This is an inspection POC for the Clearpath Robotics Husky Observer. This is not intended for commercial deployment and is just an example, but could serve as a starting point for additional development.

The purpose of this example is to create automated missions for the purpose of underpanel solar inspections such as in this video: https://www.youtube.com/watch?v=xIM0MXvQyGU. This program assumes equidistant spacing between panel rows, even terrain and limited obstacles.

<ins>Here is an outline of the features of the program: </ins>
- takes the generated map from the row generator demo as an input
- adds waypoints at the beginning and end of each row
   * - exludes the outer edges (program can be modified to include or just add extra rows)
    - starts from the row closest to and in the same direction as the first edge in the generator polygon
    - calculates heading automatically (counter can be started at 3 to reverse the direction of travel)
    - can be extended to add interpolated points on each row (example can be provided)*
- adds an automated PTZ camera rotation to the beginning waypoint of each row (the values of pan1 and pan2 can be interchanged to get the opposite effect

For more information on the Husky Observer: https://clearpathrobotics.com/husky-observer/

![image (53)](https://github.com/user-attachments/assets/54160a60-9311-4bc2-8ef1-fdf4700cff1b)
![20240417_212424](https://github.com/user-attachments/assets/4896be0f-7c0f-4290-8f7c-ff3793763020)


