cmake_minimum_required(VERSION 2.8.3)

project(internal_space_spline_trajectory_generator)

find_package(catkin REQUIRED COMPONENTS rtt_ros trajectory_msgs)

find_package(OROCOS-RTT REQUIRED)
find_package(Eigen REQUIRED)

include_directories(${catkin_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS})

orocos_component(${PROJECT_NAME} src/InternalSpaceSplineTrajectoryGenerator.cpp src/velocityprofile_spline.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
orocos_generate_package()
