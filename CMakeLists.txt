# cmake needs this line
cmake_minimum_required(VERSION 2.8)
# Enable C++11
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)
# Define project name
project(kinect)

# Find OpenCV, you may need to set OpenCV_DIR variable
# to the absolute path to the directory containing OpenCVConfig.cmake file
# via the command line or GUI
find_package(OpenCV REQUIRED)
FIND_PACKAGE(PkgConfig REQUIRED)
FIND_PACKAGE(freenect2 REQUIRED)

find_package(PCL 1.2 REQUIRED)

include_directories( ${OpenCV_INCLUDE_DIRS} )
include_directories( ${freenect2_INCLUDE_DIRS} )

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Declare the executable target built from your sources
add_executable(kinect kinect.cpp)

# Link your application with OpenCV libraries
target_link_libraries(kinect ${freenect2_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES})

