cmake_minimum_required(VERSION 3.8)
project(karstos_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(actuator_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(px4_msgs REQUIRED)

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


add_executable(vision_pose_publisher src/vision_pose_publisher.cpp)
ament_target_dependencies(vision_pose_publisher rclcpp geometry_msgs px4_msgs)

add_executable(offboard_controller src/offboard_controller.cpp)
ament_target_dependencies(offboard_controller rclcpp px4_msgs)

add_executable(px4_tf_broadcaster src/px4_tf_broadcaster.cpp)
ament_target_dependencies(px4_tf_broadcaster
  rclcpp
  px4_msgs
  tf2
  tf2_ros
  geometry_msgs
  sensor_msgs
)

install(TARGETS vision_pose_publisher offboard_controller px4_tf_broadcaster
    DESTINATION lib/${PROJECT_NAME})

# ament_target_dependencies(
#   flight_controller
#   rclcpp
#   sensor_msgs
#   geometry_msgs
#   actuator_msgs
#   std_msgs
# )
# ament_target_dependencies(pose_extractor rclcpp tf2_msgs geometry_msgs)


# target_link_libraries(flight_controller Eigen3::Eigen)

# install(TARGETS pose_extractor vision_pose_publisher
#   DESTINATION lib/${PROJECT_NAME})


ament_package()
