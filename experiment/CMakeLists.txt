

find_package(OpenCV REQUIRED)

include_directories(../include
        ${OPENCV_INCLUDE_DIR}
        )

add_executable(stereo_kitti360
        stereo_kitti360.cc
        )
target_link_libraries(stereo_kitti360
        ${PROJECT_NAME}
        )