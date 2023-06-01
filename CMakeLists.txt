cmake_minimum_required(VERSION 3.1)
project(means)

# we use 3 dimensional hypot, available after ++17 lol
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")

# SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread") # ??

# threads ?
find_package(Threads REQUIRED)

# cv
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

#serial
# Find wiringPi library
find_library(WIRINGPI_LIB wiringPi)
if(NOT WIRINGPI_LIB)
    message(FATAL_ERROR "wiringPi library not found")
endif()
# Find wiringPi dev library
find_library(WIRINGPI_DEV_LIB wiringPiDev)
if(NOT WIRINGPI_DEV_LIB)
    message(FATAL_ERROR "wiringPi dev library not found")
endif()

# apriltag
find_package(apriltag REQUIRED)
include_directories("/usr/local/include/apriltag/")

# means
add_executable(main src/main.cpp)
add_dependencies(main ${OpenCV_LIBS} stream_getter agent serial_handler gui_handler topdown)
target_link_libraries(main ${OpenCV_LIBS} stream_getter agent serial_handler gui_handler topdown)

add_library(agent src/agent.cpp)
target_link_libraries(agent ${OpenCV_LIBS} april_tag_detector pid camera_controller simple_controller)

add_library(stream_getter src/stream_getter.cpp)
target_link_libraries(stream_getter ${OpenCV_LIBS})

add_library(gui_handler src/gui_handler.cpp)
target_link_libraries(gui_handler ${OpenCV_LIBS})

add_library(serial_handler src/serial_handler.cpp)
target_link_libraries(serial_handler ${OpenCV_LIBS} ${WIRINGPI_LIB} ${WIRINGPI_DEV_LIB})

add_library(april_tag_detector src/april_tag_detector.cpp)
target_link_libraries(april_tag_detector ${OpenCV_LIBS} apriltag Threads::Threads m utils)

add_library(utils src/utils.cpp)
target_link_libraries(utils ${OpenCV_LIBS} apriltag)

add_library(topdown src/topdown.cpp)
target_link_libraries(topdown ${OpenCV_LIBS})

add_library(pid src/pid.cpp)
target_link_libraries(pid)

add_library(simple_controller src/simple_controller.cpp)
target_link_libraries(simple_controller)

add_library(camera_controller src/camera_controller.cpp)
target_link_libraries(camera_controller)

#########################TESTS###########################
# add_executable(apriltag_test test/apriltag/apriltag_test.cpp)
# target_link_libraries(apriltag_test ${OpenCV_LIBS} april_tag_detector)

# add_executable(apriltag_test2 test/apriltag/apriltag_test2.cpp)
# target_link_libraries(apriltag_test2 ${OpenCV_LIBS} april_tag_detector)

add_executable(serial_test test/serial/serial_test.cpp)
target_link_libraries(serial_test ${WIRINGPI_LIB} ${WIRINGPI_DEV_LIB})