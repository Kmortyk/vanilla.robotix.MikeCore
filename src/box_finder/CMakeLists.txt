cmake_minimum_required(VERSION 2.8.3)
project(box_finder)

find_package(catkin REQUIRED
    COMPONENTS
        actionlib
        geometry_msgs
        roscpp
        nav_msgs
        tf
        actionlib
        move_base_msgs
        std_msgs
        message_generation
)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

add_service_files(
    FILES
    GetGoal.srv
)

generate_messages(
    DEPENDENCIES
    actionlib_msgs
    geometry_msgs
    std_msgs
)

catkin_package(CATKIN_DEPENDS message_generation message_runtime roscpp std_msgs)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)


add_executable(
    move_to_table
    src/move_to_table.cpp
    src/MoveToTableNode.cpp
    src/RobotClass.cpp
    src/TrueAngle.cpp
)
add_dependencies(move_to_table ${move_to_table_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(
    move_to_table
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
)


add_executable(
    automapping
    src/automapping.cpp
    src/AutomappingNode.cpp
    src/RobotClass.cpp
    src/TrueAngle.cpp
)
add_dependencies(automapping ${automapping_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(
    automapping
    ${catkin_LIBRARIES}
)


add_executable(
    exploration_server
    src/exploration.cpp
    src/Explorer.cpp
)
add_dependencies(exploration_server ${exploration_server_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(
    exploration_server
    ${catkin_LIBRARIES}
)
