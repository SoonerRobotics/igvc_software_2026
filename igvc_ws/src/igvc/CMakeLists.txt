cmake_minimum_required(VERSION 3.8)
project(igvc)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# c++ library stuff
ament_auto_add_library(${PROJECT_NAME} SHARED
    src/node.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}
    $<INSTALL_INTERFACE:include>
)

# c++ executable stuff
ament_auto_add_executable(commander
    src/commander.cpp
)
target_link_libraries(commander ${PROJECT_NAME})

# python library stuff
ament_python_install_package(${PROJECT_NAME})

ament_auto_package()