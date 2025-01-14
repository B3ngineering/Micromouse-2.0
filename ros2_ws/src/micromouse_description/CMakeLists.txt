cmake_minimum_required(VERSION 3.8)
project(micromouse_description)

find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav_msgs REQUIRED)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# Install the launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install URDF files (if needed)
install(DIRECTORY urdf/
  DESTINATION share/${PROJECT_NAME}/urdf
)

# Install world files (if needed)
install(DIRECTORY worlds/
  DESTINATION share/${PROJECT_NAME}/worlds
)

add_executable(navigation src/navigation.cpp)
ament_target_dependencies(navigation rclcpp geometry_msgs sensor_msgs tf2_ros nav_msgs)
add_executable(flood src/flood.cpp)
ament_target_dependencies(flood rclcpp geometry_msgs sensor_msgs tf2_ros nav_msgs)
add_executable(traversal src/traversal.cpp)
ament_target_dependencies(traversal rclcpp geometry_msgs sensor_msgs tf2_ros nav_msgs)

install(TARGETS
  navigation
  flood
  traversal
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

ament_package()
