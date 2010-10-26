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

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

rosbuild_add_roslaunch_check(test/launch)

# Test test_monitor, with trans, ethercat, camera listeners               
rosbuild_add_rostest(test/launch/fake_monitor.launch)
rosbuild_add_rostest(test/launch/latch_test.launch)
rosbuild_add_rostest(test/launch/no_heartbeat_test.launch)

# Tests of camera listener
rosbuild_add_rostest(test/launch/cam_stale_test.launch)
rosbuild_add_rostest(test/launch/cam_many_warn_test.launch)
rosbuild_add_rostest(test/launch/cam_warn_test.launch)

# Test for dropped packets 
rosbuild_add_rostest(test/launch/dropped_packet_test.launch)

# Test transmission listener
# These won't work until pr2_mechanism_diagnostics is released 
# with transmission check code (dturtle).
rosbuild_add_rostest_future(test/launch/trans_error_flex.launch)
rosbuild_add_rostest_future(test/launch/trans_error_roll.launch)