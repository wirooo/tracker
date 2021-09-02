cmake_minimum_required(VERSION 3.1)
project(tracker)

find_package(OpenCV 4 REQUIRED)
set(CMAKE_EXPORT_COMPILE_COMMANDS 1)
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${PROJECT_SOURCE_DIR})


add_executable(tracker tracker.cpp projectileMotion.cpp predictor.cpp)
target_link_libraries(tracker ${OpenCV_LIBS})
