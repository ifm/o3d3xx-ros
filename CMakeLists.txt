cmake_minimum_required(VERSION 2.8.12)
project(o3d3xx)

set(CMAKE_MODULE_PATH
  "${PROJECT_SOURCE_DIR}/cmake"
  ${CMAKE_MODULE_PATH}
  )
find_package(o3d3xx_camera 0.4.5 REQUIRED)
find_package(o3d3xx_framegrabber 0.4.5 REQUIRED)
find_package(o3d3xx_image 0.4.5 REQUIRED)
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

add_message_files(
  FILES
  Extrinsics.msg
  )

add_service_files(
  FILES
  GetVersion.srv
  Dump.srv
  Config.srv
  Rm.srv
  Trigger.srv
  )

generate_messages(
  DEPENDENCIES
  std_msgs
  )

###################################
## catkin specific configuration ##
###################################
catkin_package()

#############
##  Build  ##
#############
add_compile_options(-std=c++11)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${O3D3XX_CAMERA_INCLUDE_DIRS}
  ${O3D3XX_FRAMEGRABBER_INCLUDE_DIRS}
  ${O3D3XX_IMAGE_INCLUDE_DIRS}
  )

add_executable(o3d3xx_node src/o3d3xx_node.cpp)
target_link_libraries(o3d3xx_node
  ${catkin_LIBRARIES}
  ${O3D3XX_CAMERA_LIBRARIES}
  ${O3D3XX_FRAMEGRABBER_LIBRARIES}
  ${O3D3XX_IMAGE_LIBRARIES}
  )
add_dependencies(o3d3xx_node ${PROJECT_NAME}_generate_messages_cpp)

add_executable(o3d3xx_config_node src/o3d3xx_config_node.cpp)
target_link_libraries(o3d3xx_config_node
  ${catkin_LIBRARIES}
  )
add_dependencies(o3d3xx_config_node ${PROJECT_NAME}_generate_messages_cpp)

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
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

######################
## Node-level tests ##
######################

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest(test/o3d3xx.test)
  catkin_add_nosetests(test)
endif()
