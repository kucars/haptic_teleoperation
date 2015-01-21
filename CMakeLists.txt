cmake_minimum_required(VERSION 2.8.3)
project(haptic_teleoperation)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  roscpp
  sensor_msgs
  visualization_msgs
  tf
# phantom_omni
  dynamic_reconfigure
  message_generation
  laser_geometry
  gazebo_msgs
  geometry_msgs
  #cmake_modules
 )

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED)
find_package(Eigen REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)

generate_dynamic_reconfigure_options(
 # cfg/PIDController.cfg
 # cfg/MasterController.cfg
 # cfg/SlaveController.cfg
  cfg/ForceField.cfg
 # cfg/potential_field.cfg

  #...
)


## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   TwistArray.msg
#   ContourData.msg
#   matlab_data_force.msg

#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder  nav_msgs
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   geometry_msgs  sensor_msgs nav_msgs visualization_msgs #ardrone_autonomy
)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${haptic_teleoperation}
  CATKIN_DEPENDS message_runtime nav_msgs roscpp sensor_msgs gazebo_msgs #ardrone_autonomy
  DEPENDS eigen
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)


## Declare a cpp library
add_library(
   controller
   #src/Controller.cpp
   #src/MasterController.cpp
   #src/SlaveController.cpp
   src/ForceField.cpp
)



add_executable(prf src/nodePrf.cpp src/VirtualForcePrf.cpp src/ForceField.cpp)
add_dependencies(prf ${PROJECT_NAME}_gencfg ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(prf
  #ForceField
   ${catkin_LIBRARIES}
   ${Boost_LIBRARIES}
   ${OpenCV_LIBRARIES}
)

add_executable(virtulaImpedance src/nodeVI.cpp src/VirtualImpedanceForce.cpp src/ForceField.cpp)
add_dependencies(virtulaImpedance ${PROJECT_NAME}_gencfg ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(virtulaImpedance
   ${catkin_LIBRARIES}
   ${Boost_LIBRARIES}
   ${OpenCV_LIBRARIES}
)

catkin_add_gtest(utest src/ForceFieldTests.cpp)

#src/VirtualForceBrf.cpp src/ForceField.cpp




