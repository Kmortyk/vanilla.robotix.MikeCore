#
# run like:
# $ . ./run.sh

source devel/setup.bash
export ROS_PACKAGE_PATH=~/vanilla.robotix.MikeCore:/opt/ros/melodic/share
catkin_make
rosrun lidar
