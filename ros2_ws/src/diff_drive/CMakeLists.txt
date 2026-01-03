cmake_minimum_required(VERSION 3.20)
project(diff_drive)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# Modified ------------------------------------------------------------

# Find dependencies
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

# Create an execuable for nodes
add_executable(velop src/velo_publisher.cpp) # Publisher node for velocity
add_executable(velos src/velo_subscriber.cpp) # Same situation but subscriber

# Link the dependencies format: <execuable name> <dependency1> <dependency2> ...

# IMPORTANT TO AVOID CONFUSION: ${geometry_msgs_TARGETS} simplifies to INTERFACE
# targets, not real libraries. The file "geometry_msgs/msg/Twist.msg" is a file
# for data description and not behavior, and does not produce an .so object file.

target_link_libraries (
  velop PUBLIC
  rclcpp::rclcpp
  ${geometry_msgs_TARGETS}
)

target_link_libraries (
  velos PUBLIC
  rclcpp::rclcpp
  ${geometry_msgs_TARGETS}
)

# Specify where to put the target executables
install (
  TARGETS 
    velop 
    velos
  DESTINATION lib/${PROJECT_NAME}
)

install (
  DIRECTORY
    urdf
    # meshes
    models
    worlds
    launch 
    config
  DESTINATION share/${PROJECT_NAME}
)

# ---------------------------------------------------------------------

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
