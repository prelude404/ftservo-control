cmake_minimum_required(VERSION 3.0.2)
project(ftservoControl)


set(CMAKE_CXX_FLAGS "-fPIC")
set(CMAKE_POSITION_INDEPENDENT_CODE TRUE)
# set(cv_bridge_DIR ~/catkin_make/src/cv_bridge/cmake) #nvidia

find_package(OpenCV 4.2.0 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  roscpp
  rospy
  tf
  std_msgs
  geometry_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ftservoControl
  CATKIN_DEPENDS roscpp rospy std_msgs
)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
add_definitions(${EIGEN_DEFINITIONS})

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(ftservoControl src/FEETECHservo.cpp)
# add_library(math_tools_lib src/math_tools.cpp)

target_link_libraries( ftservoControl
    ${catkin_LIBRARIES} 
    # ${PROJECT_SOURCE_DIR}/include/ftservoControl/lib_arm/libSCServo.so #nvidia
    ${PROJECT_SOURCE_DIR}/include/ftservoControl/lib_x86/libSCServo.so #laptop
    )  

add_executable(WritePos src/WritePos.cpp)
target_link_libraries(WritePos
  ${catkin_LIBRARIES}
  ftservoControl
  # ${PROJECT_SOURCE_DIR}/include/ftservoControl/lib_arm/libSCServo.so #nvidia
  ${PROJECT_SOURCE_DIR}/include/ftservoControl/lib_x86/libSCServo.so #laptop
 )
add_dependencies(WritePos ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# add_executable(servoControl src/servoControl.cpp)
# target_link_libraries(servoControl
#   ${catkin_LIBRARIES}
#   ${PROJECT_SOURCE_DIR}/include/ftservoControl/libSCServo.a
#  )
# add_dependencies(servoControl ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(initialize src/initialize.cpp)
target_link_libraries(initialize
  ${catkin_LIBRARIES}
  ftservoControl
  # ${PROJECT_SOURCE_DIR}/include/ftservoControl/lib_arm/libSCServo.so #nvidia
  ${PROJECT_SOURCE_DIR}/include/ftservoControl/lib_x86/libSCServo.so #laptop
 )
add_dependencies(initialize ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(singleServoControl src/singleServoControl.cpp)
target_link_libraries(singleServoControl
  ${catkin_LIBRARIES}
  ftservoControl
  math_tools_lib
  # ${PROJECT_SOURCE_DIR}/include/ftservoControl/lib_arm/libSCServo.so #nvidia
  ${PROJECT_SOURCE_DIR}/include/ftservoControl/lib_x86/libSCServo.so #laptop
 )
add_dependencies(initialize ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
