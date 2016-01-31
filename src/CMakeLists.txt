cmake_minimum_required(VERSION 3.0)
# Set project name
project(blocktracker)

# Set the module path
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/../cmake/")

# Find dependencies
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
find_package(Eigen3)
include_directories(${EIGEN3_INCLUDE_DIR})
find_package(AprilTags REQUIRED)
include_directories(${AprilTags_INCLUDE_DIR})

include_directories("./")

# Build the application
add_executable(blocktracker
               block_demo.cc
               block_sensor.cc
               block_tracker.cc
               frame_annotator.cc
               argos3/core/utility/math/vector3.cpp
               argos3/core/utility/math/vector2.cpp
               argos3/core/utility/math/angles.cpp
               argos3/core/utility/math/matrix/rotationmatrix3.cpp
               argos3/core/utility/math/matrix/rotationmatrix2.cpp
               argos3/core/utility/math/matrix/transformationmatrix3.cpp
               argos3/core/utility/math/matrix/transformationmatrix2.cpp)

set_property(TARGET blocktracker PROPERTY CXX_STANDARD 11)

target_link_libraries(blocktracker ${AprilTags_LIB} ${OpenCV_LIBS})
install(TARGETS blocktracker DESTINATION bin)


