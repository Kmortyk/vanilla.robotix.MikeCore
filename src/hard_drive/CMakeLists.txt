cmake_minimum_required(VERSION 2.8.12)
project(hard_drive)
set(CMAKE_CXX_STANDARD 11)
find_package(catkin REQUIRED COMPONENTS cv_bridge roscpp std_msgs sensor_msgs)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

# hard drive node
add_executable(hard_drive src/main.cpp)
target_link_libraries(hard_drive ${catkin_LIBRARIES} ${OpenCV_LIBS} opencv_core opencv_videoio opencv_highgui)

