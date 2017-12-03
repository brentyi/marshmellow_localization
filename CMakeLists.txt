cmake_minimum_required(VERSION 2.8.3)
project(marshmellow_localization)

# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  std_msgs
  tf
  pcl_msgs
  pcl_ros
  pcl_conversions
)

catkin_package(
#    INCLUDE_DIRS include
)

add_executable(detector src/detector.cpp)
target_link_libraries(detector
  ${catkin_LIBRARIES}
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


# install(TARGETS marshmellow_localization
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

