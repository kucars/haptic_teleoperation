cmake_minimum_required(VERSION 2.8.3)
project(haptic_teleoperation)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

find_path(FCL_INCLUDE_DIRS
NAMES fcl/collision.h
HINTS ${PC_FCL_INCLUDEDIR}
PATHS "${CMAKE_INSTALL_PREFIX}/include")

link_libraries(${OCTOMAP_LIBRARIES})

find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  rospy
  roscpp
  rospy
  sensor_msgs
  visualization_msgs
  tf
  phantom_omni
  dynamic_reconfigure
  message_generation
  laser_geometry
  gazebo_msgs
  geometry_msgs



 )






## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED)
find_package(Eigen REQUIRED)
find_package(PCL REQUIRED COMPONENTS )
find_package(OpenCV REQUIRED)
find_package(fcl REQUIRED)
find_package(octomap REQUIRED)


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
##add_service_files(
#   FILES
#   hmm_srv.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder  nav_msgs
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
#generate_messages(
#   DEPENDENCIES
#   geometry_msgs  sensor_msgs nav_msgs visualization_msgs  #ardrone_autonomy 
#   DEPENDS eigen 
   #PCL
#)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${haptic_teleoperation}
  CATKIN_DEPENDS message_runtime nav_msgs roscpp sensor_msgs gazebo_msgs #ardrone_autonomy
  DEPENDS eigen
  #PCL
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
  ${FCL_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
)


## Declare a cpp library
add_library(
   controller
   #src/Controller.cpp
   #src/MasterController.cpp
   #src/SlaveController.cpp
   src/ForceField.cpp
#   src/CollisionDetection.cpp
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

add_executable(brf src/nodeBrf.cpp src/VirtualForceBrf.cpp src/ForceField.cpp)
add_dependencies(brf ${PROJECT_NAME}_gencfg ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(brf
  #ForceField
   ${catkin_LIBRARIES}
   ${Boost_LIBRARIES}
   ${OpenCV_LIBRARIES}
)

#add_executable(fcl_test src/fcl_test.cpp)
#add_dependencies(fcl_test ${PROJECT_NAME}_gencfg ${PROJECT_NAME}_generate_messages_cpp)
#target_link_libraries(fcl_test
#   ${fcl_LIBRARIES}
#   ${catkin_LIBRARIES}
#   ${Boost_LIBRARIES}
#   ${OpenCV_LIBRARIES}
#)




add_executable(createOctmap src/createOctmap.cpp)
add_dependencies(createOctmap ${PROJECT_NAME}_gencfg ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(createOctmap
   ${fcl_LIBRARIES}
   ${catkin_LIBRARIES}
   ${Boost_LIBRARIES}
   ${OpenCV_LIBRARIES}
   ${OCTOMAP_LIBRARIES}
   ${PCL_INCLUDE_DIRS}
   ${PCL_LIBRARIES}

)


add_executable(controller_ros src/ControllerRos.cpp src/Controller.cpp src/SlaveController.cpp src/MasterController.cpp )
add_dependencies(controller_ros ${PROJECT_NAME}_gencfg)
target_link_libraries(controller_ros
   controller
   ${catkin_LIBRARIES}
   ${Boost_LIBRARIES}
   ${FCL_LIBRARIES}
   ${OCTOMAP_LIBRARIES}
   ${OpenCV_LIBRARIES}

) 

add_executable(CollisionDetection src/collisionDetecctionGV.cpp)
add_dependencies(CollisionDetection ${PROJECT_NAME}_gencfg)
target_link_libraries(CollisionDetection
   ${fcl_LIBRARIES}
   ${catkin_LIBRARIES}
   ${Boost_LIBRARIES}
   ${OpenCV_LIBRARIES}
   ${OCTOMAP_LIBRARIES}
   ${PCL_INCLUDE_DIRS}
   ${PCL_LIBRARIES}

) 




#add_executable(move_robot src/move_husky.cpp)
#add_dependencies(move_robot ${PROJECT_NAME}_gencfg ${PROJECT_NAME}_generate_messages_cpp)
#target_link_libraries(move_robot
# ${catkin_LIBRARIES}
# ${Boost_LIBRARIES}
#)

add_executable(robot_teleop src/robotteleop_keyboard.cpp)
target_link_libraries(robot_teleop ${catkin_LIBRARIES})



catkin_add_gtest(utest src/ForceFieldTests.cpp)






