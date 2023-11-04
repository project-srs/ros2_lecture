cmake_minimum_required(VERSION 3.8)
project(build_lecture)

add_executable(exec_sample2
  src/exec_sample2/main.cpp
  src/exec_sample2/printer.cpp
)
target_include_directories(exec_sample2
  PUBLIC include
)

install(
  TARGETS exec_sample2
  DESTINATION lib/${PROJECT_NAME}
)
