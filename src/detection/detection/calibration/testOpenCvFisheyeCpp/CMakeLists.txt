cmake_minimum_required(VERSION 3.15)
project(calibrateFisheye)

set(CMAKE_CXX_STANDARD 17)
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
find_package(nlohmann_json 3.10.0 REQUIRED)
add_executable(calibrateFisheye calibrateFisheye.cpp)
target_link_libraries(calibrateFisheye ${OpenCV_LIBS})
target_link_libraries(calibrateFisheye nlohmann_json::nlohmann_json)

