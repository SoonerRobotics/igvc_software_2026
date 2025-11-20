cmake_minimum_required(VERSION 3.8)
project(igvc_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(igvc REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_library(SOCKETCAN_LIB socketcan)
if(NOT SOCKETCAN_LIB)
  message(FATAL_ERROR "libsocketcan not found. Please install it with: sudo apt install libsocketcan-dev")
endif()

# OpenCV
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS} /usr/include/opencv4)

# Vectornav
set(VN_DIR "/usr/local/vectornav/cpp")

file(GLOB VN_SOURCES "${VN_DIR}/src/**/*.cpp")
add_library(vectornav SHARED ${VN_SOURCES})
target_include_directories(vectornav PUBLIC "${VN_DIR}/include")

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# local package include directories
include_directories(include)

# create executable (can)
add_executable(igvc_hardware_can src/can.cpp)
ament_target_dependencies(igvc_hardware_can igvc rclcpp std_msgs nav_msgs sensor_msgs cv_bridge image_transport OpenCV)
target_link_libraries(igvc_hardware_can ${OpenCV_LIBS} ${SOCKETCAN_LIB} opencv_core opencv_imgproc opencv_highgui)

# create executable (vn)
add_executable(igvc_hardware_vn src/vn.cpp)
ament_target_dependencies(igvc_hardware_vn igvc rclcpp std_msgs nav_msgs sensor_msgs)
target_link_libraries(igvc_hardware_vn vectornav)

# installing executables
install(TARGETS
   igvc_hardware_can
   igvc_hardware_vn
   DESTINATION lib/${PROJECT_NAME}
 )


ament_package()