cmake_minimum_required(VERSION 3.8)
project(odom_handler)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(std_msgs REQUIRED)
find_package(autonav_interfaces REQUIRED)
# find_package(<dependency> REQUIRED)

add_executable(wheel_odometry src/wheel_odometry.cpp src/serialib.cpp)

add_executable(wheel_odometry_publisher src/wheel_odom_pub.cpp src/serialib.cpp)
ament_target_dependencies(wheel_odometry_publisher rclcpp geometry_msgs nav_msgs tf2_ros tf2 std_msgs autonav_interfaces)

target_include_directories(wheel_odometry_publisher PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../autonav_interfaces>
  $<INSTALL_INTERFACE:include>
)

install(TARGETS
  wheel_odometry
  wheel_odometry_publisher
  DESTINATION lib/${PROJECT_NAME})

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

install(
  DIRECTORY src
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
