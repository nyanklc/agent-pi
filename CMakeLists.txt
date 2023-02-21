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

# apriltag
find_package(apriltag REQUIRED)
include_directories("/usr/local/include/apriltag/")

# means
add_executable(main src/main.cpp)
target_link_libraries(main ${OpenCV_LIBS} stream_getter agent serial_handler controls gui_handler)

add_library(agent src/agent.cpp)
target_link_libraries(agent ${OpenCV_LIBS} camera_calibrator feature_matcher controls master_object april_tag_detector)

add_library(camera_calibrator src/camera_calibrator.cpp)
target_link_libraries(camera_calibrator ${OpenCV_LIBS})

add_library(feature_matcher src/feature_matcher.cpp)
target_link_libraries(feature_matcher ${OpenCV_LIBS} master_object)

add_library(stream_getter src/stream_getter.cpp)
target_link_libraries(stream_getter ${OpenCV_LIBS})

add_library(gui_handler src/gui_handler.cpp)
target_link_libraries(gui_handler ${OpenCV_LIBS})

add_library(serial_handler src/serial_handler.cpp)
target_link_libraries(serial_handler ${OpenCV_LIBS})

add_library(controls src/controls.cpp)
target_link_libraries(controls ${OpenCV_LIBS})

add_library(master_object src/master_object.cpp)
target_link_libraries(master_object ${OpenCV_LIBS})

add_library(april_tag_detector src/april_tag_detector.cpp)
target_link_libraries(april_tag_detector ${OpenCV_LIBS} master_object apriltag Threads::Threads m utils)

add_library(utils src/utils.cpp)
target_link_libraries(utils ${OpenCV_LIBS} apriltag)

#########################TESTS###########################
add_executable(knn_test test/knn/knn_test.cpp)
target_link_libraries(knn_test ${OpenCV_LIBS} feature_matcher)

add_executable(apriltag_test test/apriltag/apriltag_test.cpp)
target_link_libraries(apriltag_test ${OpenCV_LIBS} april_tag_detector master_object)

add_executable(apriltag_test2 test/apriltag/apriltag_test2.cpp)
target_link_libraries(apriltag_test2 ${OpenCV_LIBS} april_tag_detector master_object)
