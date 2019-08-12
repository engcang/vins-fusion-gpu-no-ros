cmake_minimum_required(VERSION 2.8.3)
project(vins)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

include_directories(${CMAKE_SOURCE_DIR}/../camera_models/include)
add_library( camera_models SHARED IMPORTED )
set_target_properties(camera_models PROPERTIES IMPORTED_LOCATION ${CMAKE_SOURCE_DIR}/../camera_models/build/libcamera_models.so)


find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    geometry_msgs
    nav_msgs
    tf
    cv_bridge
    image_transport)

#find_package(OpenCV REQUIRED)
include(/usr/local/share/OpenCV/OpenCVConfig.cmake)
#message(WARNING "OpenCV_VERSION: ${OpenCV_VERSION}")

find_package(Ceres REQUIRED)

include_directories( include ${catkin_INCLUDE_DIRS} ${CERES_INCLUDE_DIRS})

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
find_package(Eigen3)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
)
catkin_package()
#message(STATUS "OpenCV library status:")
#message(STATUS "    version: ${OpenCV_VERSION}")
#message(STATUS "    libraries: ${OpenCV_LIBS}")
#message(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}")
#message(STATUS "    catkin libraries: ${catkin_LIBRARIES}")

add_library(vins_lib_gpu SHARED
    src/tools.cpp
    )
target_link_libraries(vins_lib_gpu camera_models camera_models ${OpenCV_LIBS} ${catkin_LIBRARIES}  ${CERES_LIBRARIES})

add_executable(vins_node src/rosNodeTest.cpp)
target_link_libraries(vins_node vins_lib_gpu camera_models)
