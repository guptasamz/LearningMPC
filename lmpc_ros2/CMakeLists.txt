cmake_minimum_required(VERSION 3.16)
project(lmpc_ros2 LANGUAGES CXX)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)

# Eigen/OSQP/osqp-eigen are NOT ROS packages here -- reuse the same deps
# already built for the pybind11 module (see ../src_gym/build.sh /
# README.md's Phase A). Build that first if ./../src_gym/deps is missing.
set(LMPC_DEPS_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../src_gym/deps" CACHE PATH
  "Install prefix for Eigen, OSQP, and OsqpEigen (built by src_gym/build.sh)")
list(PREPEND CMAKE_PREFIX_PATH "${LMPC_DEPS_DIR}")

find_package(Eigen3 3.4 CONFIG REQUIRED)
find_package(osqp CONFIG REQUIRED)
find_package(OsqpEigen CONFIG REQUIRED)

add_executable(lmpc_node
  src/lmpc_node.cpp
  src/main.cpp
)

# lmpc_node.cpp #includes ../src_gym/cpp/lmpc_core.cpp directly (same
# single-translation-unit pattern src_gym/CMakeLists.txt itself uses for the
# pybind11 module) -- LMPC_BUILD_PYBIND is intentionally left UNDEFINED here,
# so LMPCCore compiles with zero pybind11/Python dependency (see
# src_gym/cpp/lmpc_core.cpp's #ifdef LMPC_BUILD_PYBIND guard).
target_include_directories(lmpc_node PRIVATE
  include
  ${CMAKE_CURRENT_SOURCE_DIR}/../src_gym/cpp             # lmpc_core.cpp itself
  ${CMAKE_CURRENT_SOURCE_DIR}/../src_gym/cpp/ros_shim    # ROS/boost stand-ins (searched first)
  ${CMAKE_CURRENT_SOURCE_DIR}/../include                 # original LearningMPC headers, unmodified
  ${CMAKE_CURRENT_SOURCE_DIR}/../online_training/cpp     # residual dynamics regressor
)

target_link_libraries(lmpc_node
  OsqpEigen::OsqpEigen
  osqp::osqp
  Eigen3::Eigen
)
ament_target_dependencies(lmpc_node
  rclcpp
  nav_msgs
  ackermann_msgs
  ament_index_cpp
)
target_compile_options(lmpc_node PRIVATE
  $<$<CXX_COMPILER_ID:AppleClang,Clang,GNU>:-O3>
)

install(TARGETS lmpc_node
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch config data
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
