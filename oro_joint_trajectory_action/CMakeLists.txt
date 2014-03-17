cmake_minimum_required(VERSION 2.8.3)

project(oro_joint_trajectory_action)

find_package(catkin REQUIRED COMPONENTS rtt_ros trajectory_msgs control_msgs)

find_package(OROCOS-RTT REQUIRED)

include_directories(${catkin_INCLUDE_DIRS})

orocos_component(${PROJECT_NAME} src/JointTrajectoryAction.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
orocos_generate_package()
