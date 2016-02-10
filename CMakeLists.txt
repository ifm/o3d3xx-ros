cmake_minimum_required(VERSION 2.8.12)
project(o3d3xx)

set(CMAKE_MODULE_PATH
  "${PROJECT_SOURCE_DIR}/cmake"
  ${CMAKE_MODULE_PATH}
  )
find_package(libo3d3xx REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem)
find_package(catkin REQUIRED COMPONENTS
             image_transport
             pcl_ros
	     cv_bridge
	     roscpp
	     sensor_msgs
	     std_msgs
	     tf
	     message_generation
	     message_runtime
	     )

#######################################
## Declare ROS messages and services ##
#######################################

add_service_files(
  FILES
  GetVersion.srv
  Dump.srv
  Config.srv
  Rm.srv
  )

generate_messages(
  DEPENDENCIES
  std_msgs
  )

SET(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

###################################
## catkin specific configuration ##
###################################
catkin_package()

#############
##  Build  ##
#############

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${libo3d3xx_INCLUDE_DIRS}
  )

add_executable(o3d3xx_node src/o3d3xx_node.cpp)

target_link_libraries(o3d3xx_node
  ${catkin_LIBRARIES}
  ${libo3d3xx_LIBRARIES}
  )
add_dependencies(o3d3xx_node ${PROJECT_NAME}_generate_messages_cpp)

add_executable(o3d3xx_config_node src/o3d3xx_config_node.cpp)
target_link_libraries(o3d3xx_config_node
  ${catkin_LIBRARIES}
  )
add_dependencies(o3d3xx_config_node ${PROJECT_NAME}_generate_messages_cpp)

add_executable(o3d3xx_file_writer_node src/o3d3xx_file_writer_node.cpp)
target_link_libraries(o3d3xx_file_writer_node
  ${catkin_LIBRARIES}
  ${Boost_FILESYSTEM_LIBRARY}
  )
add_dependencies(o3d3xx_file_writer_node ${PROJECT_NAME}_generate_messages_cpp)

#############
## Install ##
#############

file(GLOB o3d3xx_launch_files launch/*.launch)
install(FILES
  ${o3d3xx_launch_files}
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  )

file(GLOB o3d3xx_rviz_files launch/*.rviz)
install(FILES
  ${o3d3xx_rviz_files}
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  )

install(TARGETS
  o3d3xx_node
  o3d3xx_config_node
  o3d3xx_file_writer_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
