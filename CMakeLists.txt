cmake_minimum_required(VERSION 2.8.3)
project(avt_vimba_camera)

find_package(catkin REQUIRED COMPONENTS
  #libvimba
  camera_info_manager
  diagnostic_updater
  driver_base
  dynamic_reconfigure
  image_geometry
  image_transport
  roscpp
  sensor_msgs
  std_msgs
  polled_camera
)

#Get architecture
set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH};${CMAKE_CURRENT_SOURCE_DIR}/cmake")
include(TargetArchitecture)
target_architecture(ARCH)

#add dynamic reconfigure api
generate_dynamic_reconfigure_options(
  cfg/AvtVimbaCamera.cfg
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS camera_info_manager diagnostic_updater driver_base dynamic_reconfigure image_geometry image_transport roscpp sensor_msgs std_msgs polled_camera
)

###########
## Build ##
###########

include_directories(
  #${libvimba_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  include
)

add_executable(avt_vimba_camera_node
  src/avt_vimba_camera_node.cpp
  src/vimba_ros.cpp
  src/frame_observer.cpp
)

add_dependencies(avt_vimba_camera_node
  ${PROJECT_NAME}_gencfg
  #${libvimba_EXPORTED_TARGETS}
)

if("${ARCH}" STREQUAL i386)
  target_link_libraries(avt_vimba_camera_node
    ${catkin_LIBRARIES}
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/32bit/libVimbaC.so
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/32bit/libVimbaCPP.so
  )
elseif("${ARCH}" STREQUAL x86_64)
  target_link_libraries(avt_vimba_camera_node
    ${catkin_LIBRARIES}
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/64bit/libVimbaC.so
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/64bit/libVimbaCPP.so
  )
else()
  message(FATAL_ERROR "[libvimba]: Architecture ${ARCH} not suported. Exiting...")
endif()



#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS avt_vimba_camera_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(FILES
  launch/avt_vimba_camera.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if("${ARCH}" STREQUAL i386)
  install(FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/32bit/libVimbaC.so
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/32bit/libVimbaCPP.so
    DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  )
elseif("${ARCH}" STREQUAL x86_64)
  install(FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/64bit/libVimbaC.so
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/64bit/libVimbaCPP.so
    DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  )
endif()
