The 'data_view' folder contains a ROS workspace for data viewing on the add-on AI computer. Copy the folder to the add-on AI computer. In a terminal, go to the 'data_view' folder and compile.
```
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

Copy the 'start_data_view.desktop' and 'start_data_view_with_route_planner.desktop' files in the 'desktop_buttons' folder to the desktop. Edit the 2 links to the 'data_view.sh' or 'data_view_with_route_planner.sh' script and 'start.png' icon in the desktop files. Right-click the desktop files and 'Allow Launching'. Double-click to launch RVIZ for base autonomy mode or route planner mode.
