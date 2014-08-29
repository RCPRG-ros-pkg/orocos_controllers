cmake_minimum_required(VERSION 2.8.3)

project(port_split)

find_package(catkin REQUIRED COMPONENTS rtt_ros cmake_modules)

find_package(OROCOS-RTT REQUIRED)
find_package(Eigen REQUIRED)

include_directories(${catkin_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS})

orocos_component(port_double_split src/PortDoubleSplit.cpp)
target_link_libraries(port_double_split ${catkin_LIBRARIES})

orocos_component(port_double_aggregate src/PortDoubleAggregate.cpp)
target_link_libraries(port_double_aggregate ${catkin_LIBRARIES})

orocos_generate_package()
