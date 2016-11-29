cmake_minimum_required(VERSION 2.8.6)
project(fake_imu)

find_package(catkin REQUIRED COMPONENTS rtt_ros sensor_msgs)

find_package(OROCOS-RTT REQUIRED)
include(${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake)

find_package(Eigen3 REQUIRED)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${USE_OROCOS_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR})
link_directories(
  ${catkin_LIBRARY_DIRS}
  ${USE_OROCOS_LIBRARY_DIRS})

orocos_component(${PROJECT_NAME} src/FakeIMU.cpp )
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

orocos_generate_package(DEPENDS rtt_ros sensor_msgs)