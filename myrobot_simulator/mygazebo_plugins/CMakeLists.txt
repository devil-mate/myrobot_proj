cmake_minimum_required(VERSION 2.8.3)
project(mygazebo_plugins)

option(ENABLE_DISPLAY_TESTS "Enable the building of tests that requires a display" OFF)

find_package(catkin REQUIRED COMPONENTS 
  gazebo_dev
  gazebo_msgs
  roscpp
  rospy
  nodelet
  angles
  geometry_msgs
  sensor_msgs
  nav_msgs
  urdf
  tf
  tf2_ros
  dynamic_reconfigure
  trajectory_msgs
  rosconsole
  polled_camera
  diagnostic_updater
  std_msgs
)

include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
  pkg_check_modules(XML libxml-2.0)
  pkg_check_modules(OGRE OGRE)
  pkg_check_modules(OGRE-Terrain OGRE-Terrain)
  pkg_check_modules(OGRE-Paging OGRE-Paging)
else()
  message(FATAL_ERROR "pkg-config is required; please install it")
endif()

find_package(Boost REQUIRED COMPONENTS thread)

execute_process(COMMAND
  pkg-config --variable=plugindir OGRE
  OUTPUT_VARIABLE OGRE_PLUGIN_PATH
  OUTPUT_STRIP_TRAILING_WHITESPACE
)

# catkin_python_setup()

# generate_dynamic_reconfigure_options(

#   cfg/Hokuyo.cfg
# )

include_directories(include
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${OGRE_INCLUDE_DIRS}
  ${OGRE-Terrain_INCLUDE_DIRS}
  ${OGRE-Paging_INCLUDE_DIRS}
)

link_directories(
  ${catkin_LIBRARY_DIRS}
  ${OGRE_LIBRARY_DIRS}
  ${OGRE-Terrain_LIBRARY_DIRS}
  ${OGRE-Paging_LIBRARY_DIRS}
)

if (NOT GAZEBO_VERSION VERSION_LESS 6.0)
  catkin_package(  INCLUDE_DIRS include LIBRARIES gazebo_ros_elevator)
endif()

if (NOT GAZEBO_VERSION VERSION_LESS 7.3)
  catkin_package(INCLUDE_DIRS include LIBRARIES gazebo_ros_harness)
endif()

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES 
  vision_reconfigure 
  gazebo_ros_utils
  CATKIN_DEPENDS
  message_runtime
  gazebo_msgs
  roscpp
  rospy
  nodelet
  angles
  geometry_msgs
  sensor_msgs
  nav_msgs
  urdf
  tf
  tf2_ros
  dynamic_reconfigure
  trajectory_msgs
  rosconsole
  std_msgs
)
# add_dependencies(${PROJECT_NAME}_gencfg ${catkin_EXPORTED_TARGETS})

## Executables
# add_executable(hokuyo_node src/hokuyo_node.cpp)
# add_dependencies(hokuyo_node ${PROJECT_NAME}_gencfg)
# target_link_libraries(hokuyo_node
#   ${catkin_LIBRARIES}
# )

# add_library(gazebo_ros_utils src/gazebo_ros_utils.cpp)
# target_link_libraries(gazebo_ros_utils ${catkin_LIBRARIES} ${Boost_LIBRARIES})

# add_library(vision_reconfigure src/vision_reconfigure.cpp)
# add_dependencies(vision_reconfigure ${PROJECT_NAME}_gencfg)
# target_link_libraries(vision_reconfigure ${catkin_LIBRARIES})



## Plugins

add_library(gazebo_ros_util src/gazebo_ros_utils.cpp)
target_link_libraries(gazebo_ros_util ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_library(RoadrobotHDrive src/roadrobotH_drive_kinetic.cpp)
target_link_libraries(RoadrobotHDrive gazebo_ros_util ${catkin_LIBRARIES} ${Boost_LIBRARIES})


#add_library(RoadrobotH_imu_sensor src/roadrobotH_imu_sensor.cpp)
#target_link_libraries(RoadrobotH_imu_sensor ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES})
#
#add_library(RoadrobotH_gps src/roadrobot_gps.cpp)
#target_link_libraries(RoadrobotH_gps ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES})
#add_dependencies(hector_gazebo_ros_gps ${PROJECT_NAME}_gencfg)

# install(TARGETS
#   
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   )

# if (NOT GAZEBO_VERSION VERSION_LESS 6.0)
#   install(TARGETS gazebo_ros_elevator
#     DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#     LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#     )
# endif()

# if (NOT GAZEBO_VERSION VERSION_LESS 7.3)
#   install(TARGETS gazebo_ros_harness
#     DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#     LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#     )
# endif()

# install(DIRECTORY include/
#    DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
#    FILES_MATCHING PATTERN "*.h"
# )

# install(PROGRAMS scripts/set_wrench.py scripts/set_pose.py scripts/gazebo_model
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#   )

# install(DIRECTORY Media
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}

# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_mygazebo_plugins.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

#   )

# install(DIRECTORY test
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#   )

# # Tests
# # These need to be run with -j1 flag because gazebo can't be run
# # in parallel.
# if (CATKIN_ENABLE_TESTING)
#   find_package(rostest REQUIRED)

#   add_rostest_gtest(set_model_state-test
#                     test/set_model_state_test/set_model_state_test.test
#                     test/set_model_state_test/set_model_state_test.cpp)
#   target_link_libraries(set_model_state-test ${catkin_LIBRARIES})

#   add_rostest(test/range/range_plugin.test)

#   


