The 'local_movement' folder contains a ROS package to move the vehicle around in short and direct movements. Copy the folder to the 'src' folder in the ROS Workspace on the vehicle NUC computer and compile.
```
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

In a terminal, launch the system on the vehicle NUC computer. In a second terminal, source the ROS workspace and launch this ROS node.
```
ros2 launch local_movement local_movement.launch
```

In a third terminal, send a 'geometry_msgs::msg::PointStamped' typed message on '/local_movement' topic to move the vehicle around locally. The movement is defined in vehicle frame. Use 'collisionStop' in the launch file to turn on and off collision stopping. During the local movement, touching any button on the joystick controller stops the vehicle.
```
ros2 topic pub --once /local_movement geometry_msgs/msg/Pose2D '{x: 0.5, y: 0.1, theta: 0.2}'
```

This node operates the vehicle in manual mode. Before the next waypoint following, the system needs to switch back to waypoint mode. Click the 'Resume Navigation to Goal' button in RVIZ, or hold the 'waypoint-mode' button on the controller and use the right joystick to set the speed. Alternatively, users can write code to send a 'ensor_msgs::msg::Joy' typed message on '/joy' topic for mode switch. Example code in C++ is below.

```
//Define joy publisher at initialization
rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr joyPublisher = node_->create_publisher<sensor_msgs::msg::Joy>("/joy", 5);
```
```
//Publish joy message before sending waypoint
sensor_msgs::msg::Joy joyMsg;

joyMsg.axes.push_back(0);
joyMsg.axes.push_back(0);
joyMsg.axes.push_back(-1.0);
joyMsg.axes.push_back(0);
joyMsg.axes.push_back(1.0);
joyMsg.axes.push_back(1.0);
joyMsg.axes.push_back(0);
joyMsg.axes.push_back(0);

joyMsg.buttons.push_back(0);
joyMsg.buttons.push_back(0);
joyMsg.buttons.push_back(0);
joyMsg.buttons.push_back(0);
joyMsg.buttons.push_back(0);
joyMsg.buttons.push_back(0);
joyMsg.buttons.push_back(0);
joyMsg.buttons.push_back(1);
joyMsg.buttons.push_back(0);
joyMsg.buttons.push_back(0);
joyMsg.buttons.push_back(0);

joyPublisher->publish(joyMsg);
```
