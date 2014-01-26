cmake_minimum_required(VERSION 2.8.3)
project(controller_common)

### ROS Dependencies ###
# Find the RTT-ROS package (this transitively includes the Orocos CMake macros)
find_package(catkin REQUIRED COMPONENTS
  rtt_ros
  rtt_roscomm
  geometry_msgs 
  std_msgs 
  message_generation
  )

find_package(Eigen REQUIRED)

add_message_files(DIRECTORY msg)
generate_messages(DEPENDENCIES geometry_msgs std_msgs)

ros_generate_rtt_typekit(controller_common)

include_directories(${catkin_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS})

### Orocos Dependencies ###
# Note that orocos_use_package() does not need to be called for any dependency
# listed in the package.xml file

include_directories(${USE_OROCOS_INCLUDE_DIRS} include)

### Orocos Targets ###

orocos_component(controller_common src/controller_common.cpp)
target_link_libraries(controller_common ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES})
add_dependencies(controller_common ${PROJECT_NAME}_gencpp)

# orocos_library(my_library src/my_library.cpp)
# target_link_libraries(my_library ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES})

# orocos_service(my_service src/my_service.cpp)
# target_link_libraries(my_service ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES})

# orocos_plugin(my_plugin src/my_plugin.cpp)
# target_link_libraries(my_plugin ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES})

# orocos_typekit(my_typekit src/my_typekit.cpp)
# target_link_libraries(my_typekit ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES})

### Orocos Package Exports and Install Targets ###

# Generate install targets for header files

orocos_install_headers(DIRECTORY include/${PROJECT_NAME})

# Export package information (replaces catkin_package() macro) 
orocos_generate_package(
  INCLUDE_DIRS include
  DEPENDS rtt_ros geometry_msgs std_msgs message_runtime rtt_roscomm
)
