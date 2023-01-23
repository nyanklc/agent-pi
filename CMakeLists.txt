cmake_minimum_required(VERSION 3.1)
project(means)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(main src/main.cpp)
target_link_libraries(main ${OpenCV_LIBS} stream_getter)

add_library(stream_getter src/stream_getter.cpp)
target_link_libraries(stream_getter ${OpenCV_LIBS})
