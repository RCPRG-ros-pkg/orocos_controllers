cmake_minimum_required(VERSION 2.8.3)

project(ec_hw_model)

find_package(catkin REQUIRED COMPONENTS common_headers rtt_ros geometry_msgs eigen_conversions)

find_package(OROCOS-RTT REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})

orocos_component(ec_hw_model src/EcHwModel.cpp src/EcDriveModel.cpp)
target_link_libraries(ec_hw_model ${catkin_LIBRARIES})

orocos_generate_package()
