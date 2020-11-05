cmake_minimum_required(VERSION 2.8)
project(MultiMapMerge)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -g")
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)
find_package(OpenCV 3 REQUIRED)
find_package(Glog REQUIRED)

include_directories(
    ${PROJECT_SOURCE_DIR}/include
    ${PROJECT_SOURCE_DIR}/3rdparty/ORB_SLAM2/include
    ${PROJECT_SOURCE_DIR}/3rdparty/ORB_SLAM2/ThirdParty/DBoW2
    ${PROJECT_SOURCE_DIR}/3rdparty/ORB_SLAM2/ThirdParty/g2o
    ${EIGEN3_INCLUDE_DIR}
    ${PCL_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${GLOG_INCLUDE_DIRS}
)

# create lib
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
add_library(${PROJECT_NAME} SHARED
    src/MapMerger.cpp
    src/DiscreteGlobalMap.cpp
    src/KeyFrameNode.cpp
)
target_link_libraries(${PROJECT_NAME}
    ${PCL_LIBRARIES}
    ${OpenCV_LIBS}
    ${GLOG_LIBRARY}
    ${PROJECT_SOURCE_DIR}/3rdparty/ORB_SLAM2/lib/libORB_SLAM2.so
)
