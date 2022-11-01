cmake_minimum_required(VERSION 2.4.6)
#include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

#rosbuild_init()

IF (NOT CMAKE_BUILD_TYPE)
    SET(CMAKE_BUILD_TYPE Release)
ENDIF ()

MESSAGE("Build type: " ${ROS_BUILD_TYPE})
MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall -march=native ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -march=native")

# Check C++11 or C++0x support
#include(CheckCXXCompilerFlag)
#CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
#CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
#if (COMPILER_SUPPORTS_CXX11)
#    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
#    add_definitions(-DCOMPILEDWITHC11)
#    message(STATUS "Using flag -std=c++11.")
#elseif (COMPILER_SUPPORTS_CXX0X)
#    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
#    add_definitions(-DCOMPILEDWITHC0X)
#    message(STATUS "Using flag -std=c++0x.")
#else ()
#    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
#endif ()
set(CMAKE_CXX_STANDARD 14)
add_definitions(-DCOMPILEDWITHC11)

#LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/../../../cmake_modules)

find_package(OpenCV 4 REQUIRED)
#if(NOT OpenCV_FOUND)
#   find_package(OpenCV 2.4.3 QUIET)
#   if(NOT OpenCV_FOUND)
#      message(FATAL_ERROR "OpenCV > 2.4.3 not found.")
#   endif()
#endif()

find_package(Eigen3 REQUIRED)
find_package(Pangolin REQUIRED)
find_package(g2o REQUIRED)
find_package(OpenMP REQUIRED)
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg cv_bridge message_filters cpp_common rosconsole rosconsole_bridge rosbag)
find_package(Boost REQUIRED COMPONENTS system serialization)


include_directories(
        ../..
        ../../include
        ../../include/CameraModels
#        ${CMAKE_CURRENT_SOURCE_DIR}/../../../Thirdparty/Sophus
        ${Pangolin_INCLUDE_DIRS}
        ${Sophus_INCLUDE_DIRS}
        ${G2O_INCLUDE_DIRS}
        ${catkin_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIRS}
)
#message(hyx:${catkin_INCLUDE_DIRS})
#message(hyx:${PROJECT_SOURCE_DIR})
#message(hyx:${catkin_LIBRARIES})

set(LIBS
        ${OpenCV_LIBS}
        ${EIGEN3_LIBS}
        ${Pangolin_LIBRARIES}
        ${catkin_LIBRARIES}

        ${Boost_LIBRARIES}
#        ${PROJECT_SOURCE_DIR}/../../../Thirdparty/DBoW2/lib/libDBoW2.so
        DBoW2_ORBSLAM3
#        ${PROJECT_SOURCE_DIR}/../../../Thirdparty/g2o/lib/libg2o.so
#        ${PROJECT_SOURCE_DIR}/../../../lib/libORB_SLAM3.so
#        ${DBoW2_LIBS}
        g2o_core g2o_stuff g2o_types_sba g2o_types_sim3 g2o_solver_csparse g2o_csparse_extension
        OpenMP::OpenMP_CXX
        ORB_SLAM3
        )

# Node for monocular camera
add_executable(Mono
        src/ros_mono.cc
        )

target_link_libraries(Mono
        ${LIBS}
        )

# Node for monocular camera (Augmented Reality Demo)
add_executable(MonoAR
        src/AR/ros_mono_ar.cc
        src/AR/ViewerAR.h
        src/AR/ViewerAR.cc
        )

target_link_libraries(MonoAR
        ${LIBS}
        )

# Node for stereo camera
add_executable(Stereo
        src/ros_stereo.cc
        )

target_link_libraries(Stereo
        ${LIBS}
        )

# Node for RGB-D camera
add_executable(RGBD
        src/ros_rgbd.cc
        )

target_link_libraries(RGBD
        ${LIBS}
        )

# Node for monocular-inertial camera
add_executable(Mono_Inertial
        src/ros_mono_inertial.cc
        )

target_link_libraries(Mono_Inertial
        ${LIBS}
        )

# Node for stereo-inertial camera
add_executable(Stereo_Inertial
        src/ros_stereo_inertial.cc
        )

target_link_libraries(Stereo_Inertial
        ${LIBS}
        )
add_executable(ros901_mono_inertial
        src/ros901_mono_inertial.cc
        )

target_link_libraries(ros901_mono_inertial
        ${LIBS}
        )

add_executable(bag901_mono_inertial
        src/bag901_mono_inertial.cc
        )

target_link_libraries(bag901_mono_inertial
        ${LIBS}
        )

add_executable(bag901_stereo
        src/bag901_stereo.cc
        )

target_link_libraries(bag901_stereo
        ${LIBS}
        )