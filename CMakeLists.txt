cmake_minimum_required(VERSION 2.8.3)
project(depth_cam_hand_tracker)

# The hand_tracking_samples requires a clang compiler
find_program(CLANGPP clang++ HINTS /usr/local/bin /usr/bin /opt/local/bin)
if(${CLANGPP} MATCHES "clang")
set(CMAKE_CXX_COMPILER ${CLANGPP})
endif(${CLANGPP} MATCHES "clang")

## Compile as C++14
add_compile_options(-std=c++14 -Wno-narrowing -Ofast -march=native -fdelayed-template-parsing)

find_package(glfw3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  human_msgs
  roslib
)

catkin_package(
  CATKIN_DEPENDS roscpp
)

include_directories(
  include
  hand_tracking_samples
  ${catkin_INCLUDE_DIRS}
)

add_executable(tracker src/tracker.cpp)
add_dependencies(tracker ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(tracker 
  ${catkin_LIBRARIES}
  glfw 
  GL 
  pthread
)
