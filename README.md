# Kinematics Model Design for ROSbot XL Robot

The goal of this checkpoint is to design a kinematics model based on the robot dimensions and transform the model into motion.

## Dependencies
 - ROS2 Humble
 - [ROSbot XL package](https://github.com/husarion/rosbot_xl_ros)

Note: after launching the simulation, confirm you can move the robot;otherwise, restart the simulation.
```
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1, y: 0.0, z: 0.0}, angular:{x: 0.0, y: 0.0, z: 0.0}}"
```

## Motion Testing
Develop the holonomic motions of the robot. These motion includes

  - move forward
  - move backward
  - move sideways to the left
  - move sideways to the right
  - turn clockwise
  - turn counter-clockwise

### Implementation
// picture showing node relations

To run these motions, use the below command.
```
ros2 launch kinematic_model kinematic_model.launch.py
```
The robot will execute each motion in order.

![test_movement](https://github.com/ptientho/ROSbot-XL-kinematics/blob/main/test_movement.gif)

## Robot Following Waypoints
Given the multiple waypoints in [dphi, dx, dy] format, the robot will follow the predefined waypoints in the absolute frame.

### Implementation
// picture showing node relations

<img src=https://github.com/ptientho/ROSbot-XL-kinematics/blob/main/eight_trajectory_waypoints.png width=400px/>

Use the following command to run the simulation.
```
ros2 launch eight_trajectory eight_trajectory.launch.py
```

// gif showing the execution

