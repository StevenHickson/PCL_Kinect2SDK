cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(PCL_Kinect2SDK)

SET_PROPERTY(GLOBAL PROPERTY TARGET_SUPPORTS_SHARED_LIBS TRUE)

IF(WIN32 OR win64)
   SET(CMAKE_FIND_LIBRARY_SUFFIXES .lib .dll)
ELSE()
   SET(CMAKE_FIND_LIBRARY_SUFFIXES .a)
ENDIF()

find_package(PCL 1.7 REQUIRED)
find_package( OpenCV REQUIRED )



add_subdirectory(src)

set(KinectSDK_INCLUDE "default value" CACHE PATH "Include directories for Kinect2 SDK")
set(KinectSDK_LIB "default value" CACHE PATH "Include directories for Kinect2 SDK")

include_directories(C:/opencv/build)
include_directories(${PCL_INCLUDE_DIRS})
include_directories(${PCL_Kinect2SDK_SOURCE_DIR}/include)
include_directories(${KinectSDK_INCLUDE})


link_directories(${PCL_LIBRARY_DIRS})
link_directories(${KinectSDK_LIB})


add_definitions(${PCL_DEFINITIONS})

file(GLOB_RECURSE PCL_Kinect2SDK_HEADERS include/*.h)
file(GLOB_RECURSE PCL_Kinect2SDK_SOURCES src/*.cpp)
set (PCL_Kinect2SDK_INCLUDE_DIRS "")
foreach (_headerFile ${PCL_Kinect2SDK_HEADERS})
    get_filename_component(_dir ${_headerFile} PATH)
    list (APPEND PCL_Kinect2SDK_INCLUDE_DIRS ${_dir})
endforeach()
list(REMOVE_DUPLICATES PCL_Kinect2SDK_INCLUDE_DIRS)

include_directories(${PCL_Kinect2SDK_INCLUDE_DIRS})
add_library(PCL_Kinect2SDK STATIC src/Microsoft_grabber2.cpp)
target_link_libraries (PCL_Kinect2SDK ${PCL_LIBRARIES} ${OpenCV_LIBS} Kinect20.lib)

add_executable (Test_SDK src/PCL_Kinect2SDK.cpp)
target_link_libraries (Test_SDK PCL_Kinect2SDK ${PCL_LIBRARIES} ${OpenCV_LIBS} Kinect20.lib)