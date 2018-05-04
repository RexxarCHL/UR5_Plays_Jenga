cmake_minimum_required(VERSION 2.8.3)
project(jenga_calibrate)

add_compile_options(-std=c++11)

find_package(Eigen3 REQUIRED)

# Eigen 3.2 (Wily) only provides EIGEN3_INCLUDE_DIR, not EIGEN3_INCLUDE_DIRS
if(NOT EIGEN3_INCLUDE_DIRS)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  tf
)

catkin_package(
  DEPENDS
  	EIGEN3
)

include_directories(${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})
add_executable(tower_location_broadcaster src/tower_location_broadcaster.cpp)
target_link_libraries(tower_location_broadcaster ${catkin_LIBRARIES})

add_executable(block_stand_location_broadcaster src/block_stand_location_broadcaster.cpp)
target_link_libraries(block_stand_location_broadcaster ${catkin_LIBRARIES})