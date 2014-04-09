find_package(PCL 1.7 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
include_directories(C:/Program Files/Microsoft SDKs/Kinect/v2.0-DevPreview1403/inc/)
include_directories(${PCL_WindowsKinectSDK_SOURCE_DIR}/include)

link_directories(${PCL_LIBRARY_DIRS})
link_directories(C:/blepo/external64/Microsoft/Kinect)

add_definitions(${PCL_DEFINITIONS})