cmake_minimum_required(VERSION 3.22)
project(igvc_camera)
set(CMAKE_CXX_STANDARD 20)

include(FetchContent)
FetchContent_Declare(spdlog GIT_REPOSITORY https://github.com/gabime/spdlog.git GIT_TAG v1.13.0)
set(SPDLOG_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(SPDLOG_BUILD_TESTS    OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(spdlog)

find_package(OpenCV REQUIRED)

add_executable(igvc_camera main.cpp)
target_include_directories(igvc_camera PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(igvc_camera PRIVATE ${OpenCV_LIBS} pthread spdlog::spdlog)

add_executable(calibrate calibrate.cpp)
target_include_directories(calibrate PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(calibrate PRIVATE ${OpenCV_LIBS})

add_executable(stereo_calibrate stereo_calibrate.cpp)
target_include_directories(stereo_calibrate PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(stereo_calibrate PRIVATE ${OpenCV_LIBS})