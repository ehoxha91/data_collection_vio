# ROS Data Collection for Kinnect2
A simple synchronized data collection for kinnect2, using ROS. 

### 

- `cd ~/catkin_ws/src/`
- `git clone https://github.com/ehoxha91/rgbd_subscriber.git`

### Build

To build do these steps:
- `cd ~/catkin_ws/`
- `catkin_make`

### Run
First:
- `cd ~/catkin_ws/src/rgbd_subscriber/`
- `source launchkinect`

Second:
- `rosrun rgbd_subscriber rgbd_subscriber_node`

Make sure you specify your data collection folder, otherwise data will be saved into catkin_ws.
