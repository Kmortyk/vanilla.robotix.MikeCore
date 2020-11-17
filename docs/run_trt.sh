# запуск без ROS
cd ~/ROS/vanilla.robotix.MikeCore/
workon tf1.15
python3 src/inference/script/trt/cmd/example_my.py

# запуск с ROS
# 1 терминал
roscore

# 2 терминал
cd ~/ROS/vanilla.robotix.MikeCore/
workon tf1.15
rosrun inference example_ros.py
