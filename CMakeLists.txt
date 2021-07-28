cmake_minimum_required(VERSION 3.10.0)
project(lidar_slam)

SET(CMAKE_BUILD_TYPE Release)
add_definitions(-msse -msse2 -msse3 -msse4 -msse4.1 -msse4.2)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS} -std=c++14 -O3 -g")
set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/" ${CMAKE_MODULE_PATH})

find_package(catkin REQUIRED COMPONENTS
        tf
        roscpp
        rospy
        tf_conversions

        pcl_ros
        pcl_conversions

        std_msgs
        sensor_msgs
        geometry_msgs
        nav_msgs
        )

find_package(PCL REQUIRED QUIET)
find_package(OpenCV REQUIRED QUIET)
find_package(OpenMP REQUIRED QUIET)
find_package(CURL REQUIRED QUIET)
find_package(PkgConfig REQUIRED QUIET)

catkin_package(
        INCLUDE_DIRS include
        DEPENDS PCL
)

include_directories(
        include
        "/usr/include/eigen3"
        ${catkin_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
        ${CURL_INCLUDE_DIRS}
)

add_definitions(${PCL_DEFINITIONS})
pkg_check_modules(JSONCPP jsoncpp)

add_library(geometryEstimate src/tools/geometryEstimate.cpp)
target_link_libraries(geometryEstimate
        ${OpenCV_LIBRARIES}
        )

add_library(geometryICP src/tools/geometryICP.cpp)
target_link_libraries(geometryICP
        ${PCL_LIBRARIES}
        )

add_executable(gicp_compus src/gicp_compus.cpp)
target_link_libraries(gicp_compus
        ${catkin_LIBRARIES}
        ${OpenCV_LIBRARIES}
        geometryICP
        geometryEstimate
        )

add_library(serialPort src/WzSerialportPlus.cpp)

add_library(curl src/curl.cpp)
target_link_libraries(curl
        CURL::libcurl
        ${JSONCPP_LIBRARIES}
        )

add_executable(global src/global_localization.cpp)
target_link_libraries(global
        ${catkin_LIBRARIES}
        ${OpenCV_LIBRARIES}
        serialPort
        curl
)