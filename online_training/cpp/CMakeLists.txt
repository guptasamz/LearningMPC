cmake_minimum_required(VERSION 3.16)
project(residual_core LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# Reuse the Eigen installed into src_gym/deps by src_gym/build.sh
list(APPEND CMAKE_PREFIX_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../../src_gym/deps")

find_package(Eigen3 3.4 CONFIG REQUIRED)
find_package(pybind11 CONFIG REQUIRED)

pybind11_add_module(residual_core residual_bindings.cpp)
target_link_libraries(residual_core PRIVATE Eigen3::Eigen)
