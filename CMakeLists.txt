cmake_minimum_required(VERSION 3.0.2)
project(line_detection)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -pthread -pg -O2 -msse3 -flto -pg -march=native -mfpmath=sse") #O0,O2: optimize 

message("CMAKE_BUILD_TYPE = ${CMAKE_BUILD_TYPE}")

IF(CMAKE_BUILD_TYPE MATCHES Debug)
  message("Debug build.")
ELSEIF(CMAKE_BUILD_TYPE MATCHES Release)
  message("Release build.")
ELSE()
  message("Some other build type.")
ENDIF()

find_package(catkin REQUIRED cmake_modules)
find_package(Eigen REQUIRED)
find_package(OpenCV REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  cv_bridge

  std_msgs
  sensor_msgs
  geometry_msgs
)

catkin_package(
  DEPENDS Eigen
  CATKIN_DEPENDS roscpp
)

set(SRC ${SRC}
  src/mono_line_detector_ros.cpp
  src/timer.cpp
  src/user_param.cpp
)

set(includeDirs ${includeDirs} ${CMAKE_CURRENT_BINARY_DIR})
set(includeDirs ${includeDirs} include)

include_directories( ${includeDirs} )
include_directories( ${catkin_INCLUDE_DIRS} )
include_directories( ${Eigen_INCLUDE_DIRS} )

# executables
add_executable (line_detector_node src/main.cpp ${SRC})
target_link_libraries (line_detector_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${Eigen_LIBRARIES})

# executables
add_executable (main_test src/main_test.cpp ${SRC})
target_link_libraries (main_test ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${Eigen_LIBRARIES})