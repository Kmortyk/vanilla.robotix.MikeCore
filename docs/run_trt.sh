# install pycuda
sudo -H pip3 install --global-option=build_ext --global-option="-I/usr/local/cuda-10.2/targets/aarch64-linux/include/" --global-option="-L/usr/local/cuda-10.2/targets/aarch64-linux/lib/" pycuda

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
