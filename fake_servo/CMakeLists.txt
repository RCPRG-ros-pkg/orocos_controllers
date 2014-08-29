cmake_minimum_required(VERSION 2.8.3)

project(fake_servo)

find_package(catkin REQUIRED COMPONENTS rtt_ros cmake_modules)

catkin_package(
  DEPENDS rtt
  CATKIN_DEPENDS rtt_ros
)

find_package(OROCOS-RTT REQUIRED)

include_directories(${catkin_INCLUDE_DIRS})

# Defines the orocos_* cmake macros. See that file for additional
# documentation.
include(${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake)
add_definitions(-DRTT_COMPONENT)
orocos_component(${PROJECT_NAME} src/FakeServo.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
orocos_generate_package()
