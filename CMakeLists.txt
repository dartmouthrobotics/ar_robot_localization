cmake_minimum_required(VERSION 2.8.3)
project(bluerov_visual_pose)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  nodelet
  pluginlib
  tf
  roslint
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES visual_pose_nodelet
  CATKIN_DEPENDS roscpp
                 nodelet
                 pluginlib
)


include_directories(include
  ${catkin_INCLUDE_DIRS}
)


add_executable(pose_node
  src/pose_node.cpp
)

add_dependencies(pose_node
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(pose_node
  ${catkin_LIBRARIES}
)


add_library(visual_pose_nodelet
  src/pose_nodelet.cpp
)

add_dependencies(visual_pose_nodelet
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(visual_pose_nodelet
  ${catkin_LIBRARIES}
)

roslint_cpp()

install(TARGETS visual_pose_nodelet pose_node
        DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(DIRECTORY plugins
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})


#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_pa1.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
