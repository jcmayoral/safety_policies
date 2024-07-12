cmake_minimum_required(VERSION 3.0.2)
project(safety_msgs)


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
   std_msgs
   geometry_msgs
   message_generation)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
add_message_files(
   DIRECTORY msg
   FILES
   PolicyFeedback.msg
   SafetyState.msg
   Object.msg
   FoundObjectsArray.msg
   RiskIndexes.msg
   RiskObject.msg
   DiagnosticArray.msg
   DiagnosticStatus.msg
   KeyValue.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   std_msgs 
   geometry_msgs
)

catkin_package(
   CATKIN_DEPENDS 
    std_msgs 
    geometry_msgs
    message_runtime
)
