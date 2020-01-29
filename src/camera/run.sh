# init ros
source /opt/ros/kinetic/setup.bash
# export working directory
export ROS_PACKAGE_PATH=~/ros/MikeCore:$ROS_PACKAGE_PATH
# run main script
rosrun mike_camera cmd.py