# Robot Controller with GUI 

This project integrates a camera feed with ROS2 to detect ArUco markers and control a TurtleBot3 robot within a Gazebo simulation. The system allows real-time robot movement based on marker detection and user interactions via the camera feed.

Prerequisites

- ROS2 Humble installed on a Linux system.
- TurtleBot3 and Gazebo simulation packages.
- Image Tools package for camera utilities.
- Python dependencies:
    - OpenCV
    - NumPy
    - cv_bridge

# Running the Project
**Terminal 1:**
Start the Camera Stream
```
source install/setup.bash
ros2 run image_tools cam2image --ros-args -r image:=/camera/image_raw
```

**Terminal 2:**
Launch the ArUco Detection Node
```
ros2 run camera_subscriber camera_node
```
To set the square size parameter:
```
ros2 run camera_subscriber camera_node --ros-args -p square_size:=10
```

**Terminal 3:**
Set Environment Variables for TurtleBot3 and Gazebo
```
export TURTLEBOT3_MODEL=burger
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:`ros2 pkg \
prefix turtlebot3_gazebo \
`/share/turtlebot3_gazebo/models/
```
Launch the Gazebo Simulation
```
ros2 launch turtlebot3_gazebo empty_world.launch.py
```

**Terminal 4:**
Run Motion Control Nodes
```
ros2 run camera_subscriber point_motion_node
```

### Screens
![obraz](https://github.com/user-attachments/assets/98b151cb-e4f6-46ae-a5e9-dd44eec2df34)
![obraz](https://github.com/user-attachments/assets/5f13ac4b-7e5c-4edc-a8c7-3c8c33b973bd)
![FilmbeztytuuWykonanozapomocClipchamp-ezgif com-video-to-gif-converter](https://github.com/user-attachments/assets/ef4f21cc-e913-49fb-bff2-2d4478a71785)





