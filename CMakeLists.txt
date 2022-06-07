cmake_minimum_required(VERSION 3.0.2)
project(icp_lidar_ceres_ros_wrapper)

if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Ceres REQUIRED)
find_package(icp_lidar_ceres REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  tf
  icp_lidar_ceres
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES icp_lidar_ros
#  CATKIN_DEPENDS sensor_msgs std_msgs icp_lidar_ceres
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  "/usr/include/eigen3"
  "/usr/local/include/knn"
)

add_executable(${PROJECT_NAME} src/icp_lidar_ceres_ros_wrapper.cpp src/icp_lidar_ceres_ros_wrapper_main.cpp)
target_link_libraries(${PROJECT_NAME} ${CERES_LIBRARIES} ${catkin_LIBRARIES})

install(TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)