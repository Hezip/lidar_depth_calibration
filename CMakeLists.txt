cmake_minimum_required(VERSION 2.8.3)
project(lidar_depth_calibration)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -g")


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  pcl_conversions
  pcl_ros
  cv_bridge
  dynamic_reconfigure
)


find_package(Threads)
find_package(PCL REQUIRED)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

#set(CMAKE_AUTOMOC ON)
#set(CMAKE_AUTOUIC ON)
#set(CMAKE_AUTORCC ON)
#set(CMAKE_INCLUDE_CURRENT_DIR ON)

list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")

generate_dynamic_reconfigure_options(
        cfg/Tutorials.cfg
)

catkin_package(
 CATKIN_DEPENDS roscpp rospy std_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(load_data src/load_data.cpp src/common.hpp)
target_link_libraries(load_data ${catkin_LIBRARIES})

add_executable(registered src/dynamic_register.cpp src/common.hpp)
target_link_libraries(registered ${catkin_LIBRARIES} ${PCL_LIBRARIES})


