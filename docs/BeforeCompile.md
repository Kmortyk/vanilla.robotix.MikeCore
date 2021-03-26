0. Initilalize lidar
sudo src/ydlidar_ros/startup/initenv.sh
1. Do BeforeCompileCameraCSI
2. Do cvBridgePython3
3. Do python3ROS
4. Install these packages (can be done via Synaptic):

ros-melodic-vision-msgs
ros-melodic-cartographer-rviz

New instructions:
0. cd vanilla.robotix.MikeCore
1. rosdep install --from-paths src --ignore-src -r -y
2. sudo src/ydlidar_ros/startup/initenv.sh
3. 
