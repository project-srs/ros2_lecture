cmake_minimum_required(VERSION 3.8)
project(rclcpp_action_lecture)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(ros2_lecture_msgs REQUIRED)

add_executable(server_node 
  src/server.cpp)
ament_target_dependencies(server_node
  rclcpp
  rclcpp_action
  ros2_lecture_msgs
)
install(TARGETS server_node
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(client_async_node 
  src/client_async.cpp)
ament_target_dependencies(client_async_node
  rclcpp
  rclcpp_action
  ros2_lecture_msgs
)
install(TARGETS client_async_node
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(client_callback_node 
  src/client_callback.cpp)
ament_target_dependencies(client_callback_node
  rclcpp
  rclcpp_action
  ros2_lecture_msgs
)
install(TARGETS client_callback_node
  DESTINATION lib/${PROJECT_NAME}
)

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
