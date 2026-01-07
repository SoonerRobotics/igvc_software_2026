cmake_minimum_required(VERSION 3.16)

project(igvc_vectornav
    VERSION 1.0
    LANGUAGES CXX
)

include(FetchContent)

# Use a modern C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Create the executable
add_executable(igvc_vectornav_exec
    src/main.cpp
)

# FlatBuffers
FetchContent_Declare(
  flatbuffers
  GIT_REPOSITORY https://github.com/google/flatbuffers.git
  GIT_TAG v25.2.10
)
set(FLATBUFFERS_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(FLATBUFFERS_BUILD_FLATC OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(flatbuffers)

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
target_include_directories(igvc_vectornav_exec
    PRIVATE ${VECTORNAV_ROOT}/include
)
target_link_libraries(igvc_vectornav_exec
    PRIVATE ${VECTORNAV_ROOT}/build/src/libVnSensor.so pthread
)

# Link FlatBuffers to the executable
target_link_libraries(igvc_vectornav_exec
    PRIVATE flatbuffers
)

# Link SpdLog to the executable
target_link_libraries(igvc_vectornav_exec
    PRIVATE spdlog::spdlog
)

# Tell the target where to find headers
target_include_directories(igvc_vectornav_exec
    PRIVATE ${PROJECT_SOURCE_DIR}/include
)