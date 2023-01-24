cmake_minimum_required(VERSION 3.1)
project(means)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread") # ??

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(main src/main.cpp)
target_link_libraries(main ${OpenCV_LIBS} stream_getter agent serial_handler controls)

add_library(agent src/agent.cpp)
target_link_libraries(agent ${OpenCV_LIBS} camera_calibrator feature_matcher controls master_object)

add_library(camera_calibrator src/camera_calibrator.cpp)
target_link_libraries(camera_calibrator ${OpenCV_LIBS})

add_library(feature_matcher src/feature_matcher.cpp)
target_link_libraries(feature_matcher ${OpenCV_LIBS} master_object)

add_library(stream_getter src/stream_getter.cpp)
target_link_libraries(stream_getter ${OpenCV_LIBS})

add_library(serial_handler src/serial_handler.cpp)
target_link_libraries(serial_handler ${OpenCV_LIBS})

add_library(controls src/controls.cpp)
target_link_libraries(controls ${OpenCV_LIBS})

add_library(master_object src/master_object.cpp)
target_link_libraries(master_object ${OpenCV_LIBS})
