The 'local_movement' folder contains a ROS package to move the vehicle around in short and direct movements. Copy the folder to the 'src' folder in the ROS Workspace on the vehicle NUC computer and compile.
```
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

In a terminal, launch the system on the vehicle NUC computer. In a second terminal, source the ROS workspace and launch this ROS node.
```
ros2 launch local_movement local_movement.launch
```

In a third terminal, send a 'geometry_msgs::msg::Pose' typed message on '/local_movement' topic to move the vehicle around locally. The movement is defined in vehicle frame by default. Set 'globalRef = true' in the launch file to use global reference. Here, only translation around x, y and rotation around z are effective. Use 'collisionStop' in the launch file to turn on and off collision stopping. Upon completion, the node sends a 'std_msgs/Bool' typed message on topic '/movement_complete' to indicate whether the local movement finishes at the goal or not.
```
ros2 topic pub --once /local_movement geometry_msgs/msg/Pose 'position:                                                                
  x: 0.5
  y: 0.1
  z: 0.0
orientation:
  x: 0.0
  y: 0.0
  z: 0.0
  w: 1.0'
```

The node operates the vehicle in manual mode. During local movement, touching any button on the joystick controller stops the vehicle. Alternatively, users can send a 'std_msgs/Bool' typed message on topic '/movement_takeover' with value true to stop the vehicle. This keeps the system in manual mode.
```
ros2 topic pub --once /movement_takeover std_msgs/msg/Bool 'data: true'
```
To stop the vehicle and switch to waypoint mode, click the 'Resume Navigation to Goal' button in RVIZ, or hold the 'waypoint-mode' button on the controller and use the right joystick to set the speed. Alternatively, users can send the message with value false. The vehicle will navigate to the current waypoint in the system.
```
ros2 topic pub --once /movement_takeover std_msgs/msg/Bool 'data: false'
```
