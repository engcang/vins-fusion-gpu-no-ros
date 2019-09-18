cmake_minimum_required(VERSION 2.8.3)
project(vins)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -fPIC -Wall")
#set(CMAKE_CXX_FLAGS_RELEASE "-O3 -fPIC -Wall -g -pthread -fopenmp")

include_directories(${CMAKE_SOURCE_DIR}/../camera_models/include)
add_library( camera_models SHARED IMPORTED )
set_target_properties(camera_models PROPERTIES IMPORTED_LOCATION  ${CMAKE_SOURCE_DIR}/../camera_models/build/libcamera_models.so)

#include(/usr/local/share/OpenCV/OpenCVConfig.cmake)
find_package(OpenCV 3 REQUIRED PATHS /usr/local/share/OpenCV NO_DEFAULT_PATH) #when manual built
include_directories(${OPENCV_INCLUDE_DIRS})
message(STATUS "    version: ${OpenCV_VERSION}") #check version

find_package(Ceres REQUIRED)
include_directories(${CERES_INCLUDE_DIRS})

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

find_package(Boost REQUIRED COMPONENTS filesystem program_options system)
include_directories(${Boost_INCLUDE_DIRS})

include_directories(include # source directory include folder
    src
    )

add_library(vio_lib_gpu SHARED
    src/tools.cpp
    )
target_link_libraries(vio_lib_gpu camera_models ${OpenCV_LIBRARIES} ${CERES_LIBRARIES})

add_executable(vins_estimator
    src/main.cpp)
target_link_libraries(vins_estimator vio_lib_gpu camera_models ${OpenCV_LIBRARIES} ${CERES_LIBRARIES})
