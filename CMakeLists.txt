cmake_minimum_required(VERSION 2.8.3)
project(ar_robot_localization)

find_package(catkin REQUIRED COMPONENTS roscpp nodelet roslint)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ar_robot_loc_nodelet
  CATKIN_DEPENDS nodelet roscpp
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(ar_robot_loc_nodelet
  src/ar_robot_loc_nodelet.cpp
)
target_link_libraries(ar_robot_loc_nodelet
  ${catkin_LIBRARIES}
)

add_executable(ar_robot_loc_node
  src/ar_robot_loc_node.cpp
)
target_link_libraries(ar_robot_loc_node
  ${catkin_LIBRARIES}
)

roslint_cpp()

install(TARGETS ar_robot_loc_node ar_robot_loc_nodelet
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(FILES nodelets.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
