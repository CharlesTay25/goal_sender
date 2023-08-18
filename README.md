
# ROS2: Goal sender

Runs on ROS2 Foxy on Ubuntu 20.04, send custom goals to /goal_pose topic



## Deployment

To build this project, run:

```bash
  cd ros2_workspace/src
  git clone https://github.com/CharlesTay25/goal_sender.git
  cd ..
  colcon build --symlink-install --packages-select goal_sender
```
After building the package, source the environment and run the package

```bash
source install/setup.bash
ros2 run goal_sender row1
```

To publish the message manually in bash, run the following:
```bash
ros2 topic pub /i069/goal_pose geometry_msgs/msg/PoseStamped "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'map'}, pose: {position: {x: -5.62584, y: -1.28362, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: -0.00452601, w: 0.99999}}}" –once
```