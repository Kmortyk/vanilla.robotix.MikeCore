cmake_minimum_required(VERSION 2.8.3)
project(jetson_camera_csi)

find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs sensor_msgs)
find_package(jetson-utils REQUIRED)
find_package(CUDA REQUIRED)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${CUDA_INCLUDE_DIRS}
)

include_directories(/usr/include/gstreamer-1.0 /usr/include/glib-2.0 /usr/include/libxml2 /usr/lib/aarch64-linux-gnu/gstreamer-1.0/include /usr/lib/x86_64-linux-gnu/gstreamer-1.0/include/ /usr/lib/aarch64-linux-gnu/glib-2.0/include/ /usr/lib/x86_64-linux-gnu/glib-2.0/include/)

# camera node
add_executable(jetson_camera_csi src/jetbot_camera.cpp src/image_converter.cpp)
target_link_libraries(jetson_camera_csi ${catkin_LIBRARIES} jetson-utils)

