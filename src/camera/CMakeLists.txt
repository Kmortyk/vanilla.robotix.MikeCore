cmake_minimum_required(VERSION 2.8.3)
project(camera)

find_package(catkin REQUIRED COMPONENTS cv_bridge roscpp rospy std_msgs sensor_msgs)
find_package(OpenCV REQUIRED)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

include_directories(
        /usr/include/gstreamer-1.0
        /usr/include/glib-2.0
        /usr/include/libxml2
        /usr/lib/aarch64-linux-gnu/gstreamer-1.0/include
        /usr/lib/x86_64-linux-gnu/gstreamer-1.0/include/
        /usr/lib/aarch64-linux-gnu/glib-2.0/include/
        /usr/lib/x86_64-linux-gnu/glib-2.0/include/
)

# camera node
add_executable(camera src/main.cpp)
target_link_libraries(camera ${catkin_LIBRARIES} ${OpenCV_LIBS})
