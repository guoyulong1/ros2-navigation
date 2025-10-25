cmake_minimum_required(VERSION 3.8)
project(robot_navigation)

# Compiler warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ------------------------------------------------------------------------------
# Dependencies
# ------------------------------------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav2_map_server REQUIRED)
find_package(spdlog REQUIRED)

# ------------------------------------------------------------------------------
# Include directories
# ------------------------------------------------------------------------------
include_directories(
  include
  include/motion_planning
  include/utils
  ${OpenCV_INCLUDE_DIRS}
)

# ------------------------------------------------------------------------------
# Executable & Sources
# ------------------------------------------------------------------------------
file(GLOB_RECURSE SRC_FILES
    src/*.cpp
    src/motion_planning/*.cpp
    src/utils/*.cpp
)

add_executable(navigation_node ${SRC_FILES})

target_link_libraries(navigation_node
  ${OpenCV_LIBS}
  spdlog::spdlog fmt::fmt
)

ament_target_dependencies(navigation_node 
  rclcpp 
  nav_msgs 
  geometry_msgs 
  sensor_msgs 
  cv_bridge 
  image_transport
  OpenCV
  visualization_msgs
)

# ------------------------------------------------------------------------------
# Install Targets
# ------------------------------------------------------------------------------

# Install the executable
install(TARGETS
  navigation_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install header files
install(
  DIRECTORY include/
  DESTINATION include
)

# Install map files
install(
  DIRECTORY maps/
  DESTINATION share/${PROJECT_NAME}/maps
)

# Install RViz and other config files
install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# Install launch files
install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# ------------------------------------------------------------------------------
# Testing (Optional)
# ------------------------------------------------------------------------------
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)  # Skip copyright check
  set(ament_cmake_cpplint_FOUND TRUE)    # Skip cpplint check
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
