cmake_minimum_required(VERSION 2.8.3)
project(ray_imu)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  sensor_msgs
  serial
  std_msgs
  tf
)

catkin_package(
)

include_directories(
  include ${catkin_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_executable(ray_imu_node
  src/ray_imu_node.cpp
)

target_link_libraries(ray_imu_node
  ${catkin_LIBRARIES}
)

install(
    TARGETS
        ray_imu_node
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)

install(DIRECTORY rviz/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rviz)
