source ~/ros_ws/vanilla.robotix.MikeCore/devel
export ROS_PACKAGE_PATH=~/ros_ws:~/ros_ws/vanilla.robotix.MikeCore/lidar/lib:/opt/ros/melodic/share
cd ../..
catkin_make
rosrun lidar
