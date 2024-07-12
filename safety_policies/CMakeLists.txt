cmake_minimum_required(VERSION 3.0.2)
project(safety_policies)
#set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fpic")
#set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(catkin REQUIRED
        COMPONENTS
        dynamic_reconfigure
        roscpp
        pluginlib
        safety_core
        visualization_msgs
        sensor_msgs
        geometry_msgs
        std_msgs
        message_filters
)

find_package(OpenCV  REQUIRED)
find_package(PCL REQUIRED)

#find_package(PkgConfig REQUIRED)
find_package(YAML-CPP 0.6 REQUIRED)

#pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)


generate_dynamic_reconfigure_options(
  cfg/ProximityPolicy.cfg
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIR}
)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES safety_policies
    CATKIN_DEPENDS
        dynamic_reconfigure
        roscpp
        pluginlib
        safety_core
        visualization_msgs
        sensor_msgs
        geometry_msgs
        std_msgs
        message_filters
)

file(GLOB POLICIES src/policies/*.cpp)
file(GLOB ACTIONS src/safe_actions/*.cpp)

add_library(safety_policies ${POLICIES} src/policies_manager.cpp)
add_library(safe_actions ${ACTIONS})
add_executable(proximity_policy_node src/proximity_policy_node.cpp)
add_executable(hri_policy_node src/hri_policy_node.cpp)

add_executable(policies_manager_node src/policies_manager_node.cpp)
add_executable(action_manager_node src/action_manager_node.cpp)

add_definitions(-DCURL_STATICLIB)

target_link_libraries(safety_policies ${catkin_LIBRARIES} curl ${JSON_INCLUDE_DIRS} ${PCL_LIBRARIES} ${YAML_CPP_LIBRARIES} yaml-cpp)
#Independent policies
target_link_libraries(proximity_policy_node safety_policies safe_actions)
target_link_libraries(hri_policy_node safety_policies safe_actions)

#Policies Manager
target_link_libraries(policies_manager_node safety_policies safe_actions)
#Action Manager
target_link_libraries(action_manager_node ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_dependencies(safety_policies ${catkin_EXPORTED_TARGETS} safety_policies_gencfg)
add_dependencies(proximity_policy_node safety_policies safe_actions safety_policies_gencfg)
add_dependencies(hri_policy_node safety_policies safe_actions safety_policies_gencfg)
add_dependencies(policies_manager_node safety_policies safe_actions safety_policies_gencfg)

### INSTALLS
install(TARGETS safety_policies safe_actions
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(TARGETS proximity_policy_node hri_policy_node
                policies_manager_node action_manager_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES policies_plugins.xml action_policies.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)
