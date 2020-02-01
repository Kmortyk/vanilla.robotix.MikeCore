#
# run like:
# $ . ./run.sh

source devel/setup.bash
export ROS_PACKAGE_PATH=~/vanilla.robotix.MikeCore:/opt/ros/melodic/share
export ROS_MASTER_URI=http://192.168.100.3:11311
catkin_make
rosrun camera camera
