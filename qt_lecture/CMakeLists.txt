cmake_minimum_required(VERSION 3.8)
project(qt_lecture)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rviz_common REQUIRED)

set(CMAKE_AUTOMOC ON)

add_executable(qt_sample1
  src/qt_sample1/main.cpp)
ament_target_dependencies(qt_sample1
  "rviz_common"
)
install(
  TARGETS qt_sample1
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(qt_sample2
  src/qt_sample2/main.cpp)
ament_target_dependencies(qt_sample2
  "rviz_common"
)
install(
  TARGETS qt_sample2
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(qt_sample3
  src/qt_sample3/main.cpp)
ament_target_dependencies(qt_sample3
  "rviz_common"
)
install(
  TARGETS qt_sample3
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(qt_sample4
  src/qt_sample4/main.cpp
  src/qt_sample4/main_dialog.cpp
)
ament_target_dependencies(qt_sample4
  "rviz_common"
)
install(
  TARGETS qt_sample4
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(qt_sample5
  src/qt_sample5/main.cpp
  src/qt_sample5/touch_widget.cpp
)
ament_target_dependencies(qt_sample5
  "rviz_common"
)
install(
  TARGETS qt_sample5
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
