include(ExternalProject)
ExternalProject_Add(g2o
        GIT_REPOSITORY https://github.com/RainerKuemmerle/g2o.git
        GIT_TAG 20201223_git
        INSTALL_DIR ${CMAKE_BINARY_DIR}/INSTALL
        CMAKE_ARGS  -DCMAKE_BUILD_TYPE=Release
        -DG2O_USE_OPENMP=ON
        -DBUILD_WITH_MARCH_NATIVE=ON
        -DCMAKE_INSTALL_PREFIX:PATH=${CMAKE_BINARY_DIR}/INSTALL
)

ExternalProject_Add(sophus
        GIT_REPOSITORY https://github.com/strasdat/Sophus.git
        GIT_TAG 1.22.10
        INSTALL_DIR ${CMAKE_BINARY_DIR}/INSTALL
        CMAKE_ARGS  -DCMAKE_BUILD_TYPE=Release
        -DBUILD_SOPHUS_EXAMPLES=OFF
        -DBUILD_SOPHUS_TESTS=OFF
        -DCMAKE_INSTALL_PREFIX:PATH=${CMAKE_BINARY_DIR}/INSTALL
        )

#ExternalProject_Add(pangolin
#        GIT_REPOSITORY https://github.com/stevenlovegrove/Pangolin.git
#        GIT_TAG v0.8
#        INSTALL_DIR ${CMAKE_BINARY_DIR}/INSTALL
#        CMAKE_ARGS
#        -DCMAKE_BUILD_TYPE=Release
#        -DBUILD_SOPHUS_EXAMPLES=OFF
#        -DBUILD_SOPHUS_TESTS=OFF
#        -DCMAKE_INSTALL_PREFIX:PATH=${CMAKE_BINARY_DIR}/INSTALL
#        )

add_custom_target(3rdparty DEPENDS g2o sophus)
