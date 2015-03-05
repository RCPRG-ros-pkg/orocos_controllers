cmake_minimum_required(VERSION 2.8.3)

project(force_control)

find_package(catkin REQUIRED COMPONENTS rtt_ros geometry_msgs eigen_conversions kdl_conversions force_control_msgs cmake_modules)

find_package(OROCOS-RTT REQUIRED)
find_package(Eigen REQUIRED)
find_package(orocos_kdl REQUIRED)

include_directories(${catkin_INCLUDE_DIRS}  ${Eigen_INCLUDE_DIRS})

orocos_component(force_control_law src/ForceControlLaw.cpp)
target_link_libraries(force_control_law ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES})
#add_dependencies(force_control_law ${catkin_EXPORTED_TARGETS})

orocos_component(force_transformation src/ForceTransformation.cpp)
target_link_libraries(force_transformation ${catkin_LIBRARIES})

orocos_component(wrench_publisher src/WrenchPublisher.cpp)
target_link_libraries(wrench_publisher ${catkin_LIBRARIES})

orocos_generate_package()
