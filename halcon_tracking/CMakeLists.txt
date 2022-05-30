cmake_minimum_required(VERSION 3.5)
project(halcon_tracking)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(interfaces REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rclcpp REQUIRED)
find_package(image_geometry REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(glfw3 REQUIRED)
find_package(glew REQUIRED)

# Find halcon here which cannot be redistributed
find_package(halcon CONFIG REQUIRED PATHS C:/Projects/Libraries/Halcon-21_11-Progress) 

# 3rd Party Libraries
add_subdirectory(src/3rd/imgui)
add_subdirectory(src/3rd/imgui_filedialog)

set(dependencies interfaces rclcpp image_geometry geometry_msgs sensor_msgs OpenCV)
set(srcs
  src/main.cpp
  src/ShapeModel.cpp
  src/SoundPlayer.cpp
)

add_executable(halcon_tracking ${srcs})
target_compile_definitions(halcon_tracking PRIVATE _SILENCE_ALL_CXX17_DEPRECATION_WARNINGS)
target_link_libraries(halcon_tracking glfw GLEW::glew imgui imgui_filedialog halcon::halcon)
ament_target_dependencies(halcon_tracking ${dependencies})
target_include_directories(halcon_tracking PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_include_directories(halcon_tracking PRIVATE
  src/3rd src/3rd/imgui)

install(TARGETS halcon_tracking
  DESTINATION lib/${PROJECT_NAME})


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

