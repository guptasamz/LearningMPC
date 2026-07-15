cmake_minimum_required(VERSION 3.16)
project(lmpc_core LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# Dependencies are pre-installed into ./deps by build.sh
# (Eigen 3.4.0, OSQP v0.6.3, osqp-eigen v0.8.1)
list(APPEND CMAKE_PREFIX_PATH "${CMAKE_CURRENT_SOURCE_DIR}/deps")

find_package(Eigen3 3.4 CONFIG REQUIRED)
find_package(osqp CONFIG REQUIRED)
find_package(OsqpEigen CONFIG REQUIRED)
find_package(pybind11 CONFIG REQUIRED)

pybind11_add_module(lmpc_core cpp/lmpc_core.cpp)

target_include_directories(lmpc_core PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/cpp/ros_shim      # ROS/boost stand-ins (searched first)
  ${CMAKE_CURRENT_SOURCE_DIR}/../include        # original LearningMPC headers, unmodified
)

target_link_libraries(lmpc_core PRIVATE OsqpEigen::OsqpEigen osqp::osqp Eigen3::Eigen)
