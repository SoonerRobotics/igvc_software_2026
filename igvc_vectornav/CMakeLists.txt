cmake_minimum_required(VERSION 3.22)
project(igvc_vectornav)

include(FetchContent)
set(CMAKE_CXX_STANDARD 20)

# Executable
add_executable(igvc_vectornav main.cpp)

# SpdLog
FetchContent_Declare(
        spdlog
        GIT_REPOSITORY https://github.com/gabime/spdlog.git
        GIT_TAG v1.13.0
)
set(SPDLOG_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(SPDLOG_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(SPDLOG_BUILD_BENCH OFF CACHE BOOL "" FORCE)
set(SPDLOG_FMT_EXTERNAL OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(spdlog)

# Vectornav
set(VECTORNAV_ROOT /usr/local/vectornav/cpp)
target_include_directories(igvc_vectornav
        PRIVATE ${VECTORNAV_ROOT}/include
)
target_link_libraries(igvc_vectornav
        PRIVATE ${VECTORNAV_ROOT}/build/src/libVnSensor.so pthread
)

# Link SpdLog to the executable
target_link_libraries(igvc_vectornav
        PRIVATE spdlog::spdlog
)
