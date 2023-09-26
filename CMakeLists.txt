cmake_minimum_required(VERSION 3.0.2)
project(robot_gui)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  robotinfo_msgs
  roscpp
  std_srvs
)

find_package(OpenCV 4 REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES robot_gui
#  CATKIN_DEPENDS geometry_msgs nav_msgs robotinfo_msgs roscpp std_srvs
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_library(robot_gui src/robot_gui.cpp)
add_dependencies(robot_gui ${robot_gui_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(robot_gui ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(robot_gui_node src/robot_gui_node.cpp)
add_dependencies(robot_gui_node ${robot_gui_node_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(robot_gui_node ${catkin_LIBRARIES} robot_gui)

#############
## Install ##
#############
install(TARGETS robot_gui_node
		RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
