cmake_minimum_required(VERSION 2.8.3)
project(kuri_usar_teleoperation)


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  geometry_msgs
  dynamic_reconfigure
  cmake_modules
)
find_package(Eigen REQUIRED)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)


 
catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

## Declare a C++ executable
# add_executable(sar_haptic_node src/sar_haptic_node.cpp)
add_executable(teleop_uav src/sarKeyTeleop.cpp)
add_dependencies(teleop_uav ${PROJECT_NAME}_gencfg ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(teleop_uav ${catkin_LIBRARIES} )

add_executable(teleop_uav_Data_collection_class_1 src/dataCollectionController.cpp)
add_dependencies(teleop_uav_Data_collection_class_1 ${PROJECT_NAME}_gencfg ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(teleop_uav_Data_collection_class_1 ${catkin_LIBRARIES} )

