cmake_minimum_required(VERSION 3.10)
project(syn_pf_cpp)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

add_compile_options(-Wall -Wextra -Wpedantic -O3 -march=native)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# CUDA support — enables rmgpu range method
option(WITH_CUDA "Enable CUDA rmgpu range method" ON)
if(WITH_CUDA)
  if(NOT CMAKE_CUDA_COMPILER)
    set(CMAKE_CUDA_COMPILER "/usr/local/cuda/bin/nvcc")
  endif()
  enable_language(CUDA)
  find_package(CUDAToolkit REQUIRED)
  set(CMAKE_CUDA_ARCHITECTURES 87)  # Jetson Orin
  message(STATUS "syn_pf_cpp: CUDA enabled (arch ${CMAKE_CUDA_ARCHITECTURES})")
endif()

# rangelib headers (header-only ranges:: namespace).
# RangeLib.h uses both "vendor/..." and "includes/..." relative includes, plus we want
# bare `#include "RangeLib.h"`. So expose both /range_libc and /range_libc/includes.
set(RANGELIB_ROOT "/home/f1tenth/range_libc" CACHE PATH "rangelib root")

set(SYNPF_SRC
  src/synpf_component.cpp
  ${RANGELIB_ROOT}/vendor/lodepng/lodepng.cpp
)
if(WITH_CUDA)
  list(APPEND SYNPF_SRC ${RANGELIB_ROOT}/includes/kernels.cu)
endif()

add_library(synpf_component SHARED ${SYNPF_SRC})
target_include_directories(synpf_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${RANGELIB_ROOT}
  ${RANGELIB_ROOT}/includes
  ${EIGEN3_INCLUDE_DIR}
)
ament_target_dependencies(synpf_component
  rclcpp rclcpp_components
  sensor_msgs nav_msgs geometry_msgs
  tf2 tf2_ros tf2_geometry_msgs
)
target_compile_definitions(synpf_component PRIVATE
  "SYN_PF_CPP_BUILDING_DLL"
)
if(WITH_CUDA)
  target_compile_definitions(synpf_component PRIVATE
    USE_CUDA=1
    CHUNK_SIZE=262144
    NUM_THREADS=256)
  set_source_files_properties(${RANGELIB_ROOT}/includes/kernels.cu
    PROPERTIES LANGUAGE CUDA)
  target_link_libraries(synpf_component CUDA::cudart)
endif()

rclcpp_components_register_nodes(synpf_component "syn_pf_cpp::SynPFComponent")

# Standalone executable wrapper (for non-composable launch)
add_executable(synpf_node src/synpf_node_main.cpp)
target_link_libraries(synpf_node synpf_component)
ament_target_dependencies(synpf_node rclcpp)

# Latency probe component
add_library(latency_probe_component SHARED src/latency_probe.cpp)
target_include_directories(latency_probe_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(latency_probe_component
  rclcpp rclcpp_components geometry_msgs)
rclcpp_components_register_nodes(latency_probe_component "syn_pf_cpp::LatencyProbe")

add_executable(latency_probe src/latency_probe_main.cpp)
target_link_libraries(latency_probe latency_probe_component)
ament_target_dependencies(latency_probe rclcpp)

install(TARGETS synpf_component latency_probe_component
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS synpf_node latency_probe
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/
  DESTINATION include
)
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

# scripts/compare_pf_node.py (upstream's Python-PF comparison/debug tool)
# intentionally not vendored here -- not needed for this package's sim/real
# usage. Upstream's own install(PROGRAMS scripts/compare_pf_node.py ...)
# rule is dropped to match (it would otherwise fail cmake --install with a
# missing-file error, since the referenced script doesn't exist in this tree).

ament_package()
