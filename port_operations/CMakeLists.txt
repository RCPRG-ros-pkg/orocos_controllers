cmake_minimum_required(VERSION 2.8.3)

project(port_operations)

find_package(catkin REQUIRED COMPONENTS rtt_ros std_msgs)

find_package(OROCOS-RTT REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})

orocos_component(im2rm src/IM2RM.cpp)
target_link_libraries(im2rm ${catkin_LIBRARIES})

orocos_component(port_double_split src/PortDoubleSplit.cpp)
target_link_libraries(port_double_split ${catkin_LIBRARIES})

orocos_component(port_double_aggregate src/PortDoubleAggregate.cpp)
target_link_libraries(port_double_aggregate ${catkin_LIBRARIES})

orocos_component(port_double_sum src/PortDoubleSum.cpp)
target_link_libraries(port_double_sum ${catkin_LIBRARIES})

orocos_component(port_double_to_float64 src/PortDoubleToFloat64.cpp)
target_link_libraries(port_double_to_float64 ${catkin_LIBRARIES})

orocos_generate_package()
