cmake_minimum_required(VERSION 3.22 FATAL_ERROR)

project(movingleastsquares)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
include_directories(include) # Add your include directory
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Update the add_executable line
add_executable(movingleastsquares src/main.cpp src/MovingLeastSquares.cpp)
target_link_libraries(movingleastsquares ${PCL_LIBRARIES})
