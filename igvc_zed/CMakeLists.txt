cmake_minimum_required(VERSION 3.22)
project(igvc_zed)
include(FetchContent)
set(CMAKE_CXX_STANDARD 20)

# ─── Executable ───────────────────────────────────────────────────────────────
add_executable(igvc_zed main.cpp)

# ─── SpdLog ───────────────────────────────────────────────────────────────────
FetchContent_Declare(
        spdlog
        GIT_REPOSITORY https://github.com/gabime/spdlog.git
        GIT_TAG v1.13.0
)
set(SPDLOG_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(SPDLOG_BUILD_TESTS    OFF CACHE BOOL "" FORCE)
set(SPDLOG_BUILD_BENCH    OFF CACHE BOOL "" FORCE)
set(SPDLOG_FMT_EXTERNAL   OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(spdlog)

# ─── ZED SDK ─────────────────────────────────────────────────────────────────
# The ZED SDK ships its own CMake config; find_package locates it automatically
# after a standard SDK install (/usr/local/zed).
find_package(ZED 4 REQUIRED)
find_package(CUDA REQUIRED)   # ZED SDK requires CUDA

target_include_directories(igvc_zed PRIVATE ${ZED_INCLUDE_DIRS} ${CUDA_INCLUDE_DIRS})

target_link_libraries(igvc_zed
        PRIVATE
        ${ZED_LIBRARIES}
        ${CUDA_LIBRARIES}
        pthread
        spdlog::spdlog
)

# ─── Compiler options ─────────────────────────────────────────────────────────
target_compile_options(igvc_zed PRIVATE -O2 -Wall -Wextra)