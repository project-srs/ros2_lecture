cmake_minimum_required(VERSION 3.8)
project(sqlite_lecture)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs QUIET)
find_package(SQLite3 QUIET)

include_directories(my_target
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

add_executable(sample1
  src/sample1/main.cpp)
ament_target_dependencies(sample1
  "rclcpp"
  "SQLite3"
)
install(
  TARGETS sample1
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(sample2
  src/sample2/main.cpp)
ament_target_dependencies(sample2
  "rclcpp"
  "SQLite3"
)
install(
  TARGETS sample2
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(sample3
  src/sample3/main.cpp)
ament_target_dependencies(sample3
  "rclcpp"
  "SQLite3"
)
install(
  TARGETS sample3
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(sample4
  src/sample4/main.cpp)
ament_target_dependencies(sample4
  "rclcpp"
  "geometry_msgs"
  "SQLite3"
)
install(
  TARGETS sample4
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
