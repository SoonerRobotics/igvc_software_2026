cmake_minimum_required(VERSION 3.8)
project(igvc_display)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(igvc REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# creating executables
add_executable(camera_adapter src/camera_adapter.cpp)
ament_target_dependencies(camera_adapter rclcpp std_msgs sensor_msgs igvc)

target_include_directories(camera_adapter PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_executable(data_adapter src/data_adapter.cpp)
ament_target_dependencies(data_adapter rclcpp std_msgs sensor_msgs igvc)

target_include_directories(data_adapter PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# installing executables
install(TARGETS
   camera_adapter data_adapter
   DESTINATION lib/${PROJECT_NAME}
)

ament_package()