cmake_minimum_required(VERSION 2.8.3)
project(text_locator)

find_package(catkin REQUIRED COMPONENTS cv_bridge image_transport message_filters openni_launch roscpp sensor_msgs std_msgs geometry_msgs message_generation pcl_ros )

#######################################
## Declared ROS message              ##
#######################################

## Generate messages in the 'msg' folder
 add_message_files(
   FILES
   TextLocation.msg
 )

## Generate added messages and services with any dependencies listed here
 generate_messages(
  DEPENDENCIES
   std_msgs
   geometry_msgs
 )


catkin_package(
# INCLUDE_DIRS include
  LIBRARIES CCV TESSERACT PCL OpenCV
  CATKIN_DEPENDS cv_bridge image_transport message_filters openni_launch pcl_ros roscpp sensor_msgs std_msgs geometry_msgs message_runtime
)

###########
## Build ##
###########

#for eclipse indexer
#find_path(STDIO_INCLUDE_PATH stdio.h)
#include_directories("${STDIO_INCLUDE_PATH}/dummy/../")

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/text_locator/cmake/Modules/")

find_package(TESSERACT REQUIRED)
include_directories(${TESSERACT_INCLUDE_DIR})
set(LIBS ${LIBS} ${TESSERACT_LIBRARIES})

find_package(CCV REQUIRED)
include_directories(${CCV_INCLUDE_DIR})
set(LIBS ${LIBS} ${CCV_LIBRARY})

find_package(PCL 1.6 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS}) 
link_directories(${PCL_LIBRARY_DIRS}) 
add_definitions(${PCL_DEFINITIONS})

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

message(STATUS "linked libs: ${LIBS}")

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

set(SOURCES src/TRecognizer.cpp src/TextLocator.cpp src/TDetector.cpp src/main.cpp)

add_executable(text_locator_node ${SOURCES})

target_link_libraries(text_locator_node ${LIBS} ${PCL_LIBRARIES} ${catkin_LIBRARIES} ${OpenCV_LIBS})

#############
## Install ##
#############

install(TARGETS text_locator_node
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

