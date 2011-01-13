cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()


find_library(HID_LIBRARY hid)
if (HID_LIBRARY)
  #common commands for building c++ executables and libraries
  rosbuild_add_library(${PROJECT_NAME} src/libdelcom_usb_light/delcom_usb_light.cpp)
  #target_link_libraries(${PROJECT_NAME} another_library)
  #rosbuild_add_boost_directories()
  #rosbuild_link_boost(${PROJECT_NAME} thread)
  rosbuild_add_executable(delcom_usb_light src/utilities/delcom_util.cpp)
  target_link_libraries(delcom_usb_light ${PROJECT_NAME} hid)
  
  rosbuild_add_executable(wgtest_indicator src/nodes/wgtest_status_indicator.cpp)
  target_link_libraries(wgtest_indicator ${PROJECT_NAME} hid)
else()
  message(STATUS "HID_LIBRARY not found! Disabling build! HAHAHAHAHA")
endif()
