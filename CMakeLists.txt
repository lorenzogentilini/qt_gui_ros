cmake_minimum_required(VERSION 3.0.2)
project(qt_gui)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS} -O3 -Wall")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  mavros_msgs
  sensor_msgs
  cv_bridge
  tf2_ros
  mav_planning
)

find_package(OpenCV REQUIRED)
find_package(Qt5Core REQUIRED)
find_package(Qt5Widgets REQUIRED)
find_package(Qt5Quick REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES qt_gui
  CATKIN_DEPENDS roscpp rospy std_msgs
  DEPENDS OpenCV
)

qt5_wrap_cpp(QT_MOC_HPP include/qt_gui/ControlWindow.hpp
						            include/qt_gui/RobotThread.hpp)

include_directories(
#  include
  ${catkin_INCLUDE_DIRS}
  ${Qt5Core_INCLUDE_DIRS}
  ${Qt5Widgets_INCLUDE_DIRS}
  ${Qt5Quick_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/include/qt_gui
)

## Declare a C++ library
add_library(${PROJECT_NAME}
  src/ControlWindow.cpp
  src/RobotThread.cpp
)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${QT_LIBRARIES}
  ${OpenCV_LIBS}
  Qt5::Widgets
  Qt5::Core
  Qt5::Quick
)

## Declare a C++ executable
add_executable(qt_gui_node src/qt_gui_main.cpp ${QT_RESOURCES_CPP} ${QT_MOC_HPP})

## Specify libraries to link a library or executable target against
target_link_libraries(qt_gui_node
  ${catkin_LIBRARIES}
  ${QT_LIBRARIES}
  ${OpenCV_LIBS}
  Qt5::Widgets
  Qt5::Core
  Qt5::Quick
  qt_gui
)