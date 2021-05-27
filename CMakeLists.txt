cmake_minimum_required(VERSION 2.8...3.19)

project(cv_mp CXX)

# enable c++ 11
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# take respective flags for debug & release process
set(CMAKE_CXX_FLAGS_DEBUG_INIT "-std=c++11 -g -fno-omit-frame-pointer")
set(CMAKE_CXX_FLAGS_RELEASE_INIT "-std=c++11 -g -O3")

# default build type : Debug
if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug)
endif ()

# import MKL
set(MKL_INCLUDE_DIRS "/opt/intel/oneapi/mkl/latest/include")
set(MKL_LIB_DIRS "/opt/intel/oneapi/mkl/latest/lib/intel64")

# import Eigen3
find_package(Eigen3 REQUIRED)

# import OpenCV
find_package(OpenCV REQUIRED)

# import glog
include(./cmake/glog.cmake)

# make all visible
include_directories(
        SYSTEM
        src
        ${Boost_INCLUDE_DIR}
        ${EIGEN3_INCLUDE_DIRS}
        ${GLOG_INCLUDE_DIRS}
        ${MKL_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
)

link_directories(
        /user/local/lib
        ${GLOG_LIBRARIES}
        ${MKL_LIB_DIRS}
)

# src
add_subdirectory(src)

# test
enable_testing()
add_subdirectory(tests)