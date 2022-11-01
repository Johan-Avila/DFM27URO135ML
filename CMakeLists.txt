cmake_minimum_required(VERSION 3.0.2)
project(dfm27uro135ml)

set(CMAKE_BUILD_TYPE "Release")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g -pthread")

find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  # msg
  std_msgs
  sensor_msgs
)

find_package(PkgConfig REQUIRED)
pkg_check_modules(GSTREAMER REQUIRED gstreamer-1.0 gstreamer-app-1.0 gstreamer-video-1.0)
pkg_check_modules(TCAMLIB REQUIRED tcam)

catkin_package(
  INCLUDE_DIRS include

  CATKIN_DEPENDS 
  std_msgs
  sensor_msgs 
)

# include directories
include_directories(
  include
  ${catkin_INCLUDE_DIRS}  
  ${GSTREAMER_INCLUDE_DIRS}
  ${TCAMLIB_INCLUDE_DIRS}
)

# libraries
link_libraries(
  ${catkin_LIBRARIES}
  ${TCAMLIB_LIBRARIES} 
  ${GSTREAMER_LIBRARIES}
)

add_executable(${PROJECT_NAME}_ListDevices src/ListDevices.cpp)
target_link_libraries(${PROJECT_NAME}_ListDevices ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}_ListFormats src/ListFormats.cpp)
target_link_libraries(${PROJECT_NAME}_ListFormats ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}_ListProperties src/ListProperties.cpp)
target_link_libraries(${PROJECT_NAME}_ListProperties ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}_LiveStreamCam src/LiveStreamCam.cpp)
target_link_libraries(${PROJECT_NAME}_LiveStreamCam ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}_PublisherImg src/PublisherImg.cpp)
target_link_libraries(${PROJECT_NAME}_PublisherImg ${catkin_LIBRARIES})