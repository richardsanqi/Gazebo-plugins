cmake_minimum_required(VERSION 3.0.2)
add_compile_options(-std=c++11)
find_package(roscpp REQUIRED)
find_package(std_msgs REQUIRED)
include_directories(${roscpp_INCLUDE_DIRS})
include_directories(${std_msgs_INCLUDE_DIRS})
# Find Gazebo
find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")
# Build our plugin
add_library(position_controller SHARED position_controller.cc)
target_link_libraries(position_controller ${GAZEBO_LIBRARIES} ${roscpp_LIBRARIES})
add_library(velodyne_plugin SHARED velodyne_plugin.cc)
target_link_libraries(velodyne_plugin ${GAZEBO_LIBRARIES})

add_library(set_speed_joint_plugin SHARED set_speed_joint_plugin.cpp)
target_link_libraries(set_speed_joint_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_executable(vel vel.cc)
target_link_libraries(vel ${GAZEBO_LIBRARIES})


