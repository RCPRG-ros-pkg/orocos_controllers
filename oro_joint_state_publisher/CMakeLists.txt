cmake_minimum_required(VERSION 2.8.3)

project(oro_joint_state_publisher)

find_package(catkin REQUIRED COMPONENTS rtt_ros sensor_msgs)

find_package(Eigen REQUIRED)

include_directories(${catkin_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS})

orocos_component(${PROJECT_NAME} src/JointStatePublisher.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

orocos_generate_package(
  DEPENDS rtt_ros sensor_msgs
)


