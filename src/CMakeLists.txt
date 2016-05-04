cmake_minimum_required(VERSION 3.0)
# Set project name
project(blocktracker)

# Set the module path
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/../cmake/")

# Set debug build
set(CMAKE_BUILD_TYPE Debug)

include_directories("./argos3")

# Find dependencies
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
find_package(AprilTag REQUIRED)
include_directories(${AprilTag_INCLUDE_DIR})

# Build the application
add_executable(blocktracker
               block_demo.cc
               block_sensor.cc
               block_tracker.cc
               frame_annotator.cc
               hungarian_solver.cc               
               tag.cc
               argos3/core/utility/math/angles.cpp
               argos3/core/utility/math/vector2.cpp
               argos3/core/utility/math/vector3.cpp
               argos3/core/utility/math/matrix/rotationmatrix3.cpp
               argos3/core/utility/math/matrix/transformationmatrix3.cpp)

set_property(TARGET blocktracker PROPERTY CXX_STANDARD 11)

message("AprilTag_LIB = ${AprilTag_LIB}")

target_link_libraries(blocktracker ${AprilTag_LIB} ${OpenCV_LIBS})
install(TARGETS blocktracker DESTINATION bin)


