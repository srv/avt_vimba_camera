cmake_minimum_required(VERSION 2.8.3)
project(avt_vimba_camera)

find_package(catkin REQUIRED COMPONENTS
  #libvimba
  camera_info_manager
  diagnostic_updater
  dynamic_reconfigure
  image_geometry
  image_transport
  message_filters
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
  cfg/AvtVimbaCameraStereo.cfg
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS camera_info_manager diagnostic_updater dynamic_reconfigure image_geometry image_transport roscpp sensor_msgs std_msgs polled_camera
)

###########
## Build ##
###########

include_directories(
  #${libvimba_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  include
)

function(add_dependencies_and_linkings arg)
  add_dependencies(${arg}
    ${PROJECT_NAME}_gencfg
    #${libvimba_EXPORTED_TARGETS}
  )

  if("${ARCH}" STREQUAL i386)
    target_link_libraries(${arg}
      ${catkin_LIBRARIES}
      ${CMAKE_CURRENT_SOURCE_DIR}/lib/32bit/libVimbaC.so
      ${CMAKE_CURRENT_SOURCE_DIR}/lib/32bit/libVimbaCPP.so
    )
  elseif("${ARCH}" STREQUAL x86_64)
    target_link_libraries(${arg}
      ${catkin_LIBRARIES}
      ${CMAKE_CURRENT_SOURCE_DIR}/lib/64bit/libVimbaC.so
      ${CMAKE_CURRENT_SOURCE_DIR}/lib/64bit/libVimbaCPP.so
    )
  elseif("${ARCH}" STREQUAL armv7)
    target_link_libraries(${arg}
      ${catkin_LIBRARIES}
      ${CMAKE_CURRENT_SOURCE_DIR}/lib/arm_32bit/libVimbaC.so
      ${CMAKE_CURRENT_SOURCE_DIR}/lib/arm_32bit/libVimbaCPP.so
    )
  else()
    message(FATAL_ERROR "[libvimba]: Architecture ${ARCH} not suported. Exiting...")
  endif()
endfunction(add_dependencies_and_linkings)

add_executable(mono_camera_node
  src/nodes/mono_camera_node.cpp
  src/mono_camera.cpp
  src/avt_vimba_camera.cpp
  src/frame_observer.cpp
)

add_dependencies_and_linkings(mono_camera_node)

add_executable(stereo_camera_node
  src/nodes/stereo_camera_node.cpp
  src/stereo_camera.cpp
  src/avt_vimba_camera.cpp
  src/frame_observer.cpp
)

add_dependencies_and_linkings(stereo_camera_node)

add_executable(sync_node
  src/nodes/sync_node.cpp
  src/sync.cpp
)

add_dependencies_and_linkings(sync_node)

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS mono_camera_node stereo_camera_node
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
  launch/mono_camera.launch
  launch/stereo_camera_one_node.launch
  launch/stereo_camera_two_nodes.launch
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
elseif("${ARCH}" STREQUAL armv7)
  install(FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/arm_32bit/libVimbaC.so
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/arm_32bit/libVimbaCPP.so
    DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  )
endif()

