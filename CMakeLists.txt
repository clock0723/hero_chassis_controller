cmake_minimum_required(VERSION 3.10)
project(ros_package_template)

## Compile as C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## Enable compiler warnings
add_definitions(-Wall -Werror)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  controller_interface
  hardware_interface
  pluginlib
  roscpp
  roslint
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    controller_interface
    hardware_interface
    pluginlib
    roscpp
)

###########
## Build ##
###########
include_directories(
  include
  include/hero_chassis_controller
  ${catkin_INCLUDE_DIRS}
)

## Build the controller plugin library
add_library(${PROJECT_NAME}
  src/hero_chassis_controller.cpp
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

if(catkin_EXPORTED_TARGETS)
  add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
endif()

#############
## Install ##
#############
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/hero_chassis_controller/
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}/hero_chassis_controller
  FILES_MATCHING PATTERN "*.h"
)

install(DIRECTORY doc
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

##########################
## Static code analysis ##
##########################
if(CATKIN_ENABLE_TESTING)
  roslint_cpp()
endif()
