Ros-version: ros2 Humble 

## Building: 
Copy the packages from `src/` to your workspace's `src/`.

## Running
To execute the simulation, run the following (after building and sourcing the workspace):


robot rotation: 
```
ros2 launch aruco_marker_detector multi_launch.py
```

camera rotation:
```
ros2 launch aruco_marker_detector camera_rotation_multi_launch.py
```
