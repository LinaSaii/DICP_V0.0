#==============================================================================
# CMake Build Configuration for Doppler ICP Stitcher Open3D Professional Edition
#==============================================================================
# This file defines the build system configuration for professional-grade
# point cloud registration with CUDA acceleration and Open3D integration.
#==============================================================================
#-------------------------------------------------------------------------------
# Project Foundation and Language Standards
#-------------------------------------------------------------------------------
cmake_minimum_required(VERSION 3.8)
project(doppler_icp_stitcher_open3d_pro2)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

#-------------------------------------------------------------------------------
# GPU Acceleration Configuration
#-------------------------------------------------------------------------------

# Enable CUDA
enable_language(CUDA)

# Find CUDA
find_package(CUDAToolkit REQUIRED)


#-------------------------------------------------------------------------------
# ROS 2 Core Dependencies
#-------------------------------------------------------------------------------
# Find ROS2 dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rosbag2_cpp REQUIRED)

#-------------------------------------------------------------------------------
# Mathematical and Computational Dependencies
#-------------------------------------------------------------------------------
# Find Eigen3
find_package(Eigen3 REQUIRED)

#-------------------------------------------------------------------------------
# Open3D Point Cloud Processing Framework
#-------------------------------------------------------------------------------

# Open3D
find_package(Open3D QUIET)
if(NOT Open3D_FOUND)
    find_path(Open3D_INCLUDE_DIRS NAMES open3d/Open3D.h PATHS /usr/local/include /usr/include)
    find_library(Open3D_LIBRARIES NAMES Open3D open3d PATHS /usr/local/lib /usr/lib)
    if(Open3D_INCLUDE_DIRS AND Open3D_LIBRARIES)
        set(Open3D_FOUND TRUE)
    else()
        message(FATAL_ERROR "Open3D not found")
    endif()
endif()

#-------------------------------------------------------------------------------
# Executable Target Definition
#-------------------------------------------------------------------------------

add_executable(stitch_node
    src/main.cpp
    src/doppler_icp_stitcher.cpp
    src/temporal_persistence_filter.cpp
    src/async_preprocessor.cpp
    src/utils.cpp
    src/cuda_acceleration.cu
)

#-------------------------------------------------------------------------------
# CUDA Configuration and GPU Architecture Targeting
#-------------------------------------------------------------------------------
set_target_properties(stitch_node PROPERTIES
    CUDA_RUNTIME_LIBRARY Shared
    CUDA_ARCHITECTURES "61"   # GTX 10xx
)

#-------------------------------------------------------------------------------
# Performance Optimization with Numerical Stability Preservation
#-------------------------------------------------------------------------------
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(stitch_node PRIVATE
        -O2                    # Use O2 instead of O3 (safer for numerical code)
        -march=native          # Still use native architecture
        -funroll-loops         # Safe loop unrolling
        -ftree-vectorize       # Safe vectorization
    )
    
endif()
#-------------------------------------------------------------------------------
# Performance Optimization with Numerical Stability Preservation
#-------------------------------------------------------------------------------
if(CMAKE_CXX_COMPILER_ID STREQUAL "MSVC")
    target_compile_options(stitch_node PRIVATE /O2 /arch:AVX2)
    
endif()

#-------------------------------------------------------------------------------
# Include Directory Configuration
#-------------------------------------------------------------------------------
target_include_directories(stitch_node PRIVATE
    ${EIGEN3_INCLUDE_DIRS}
    ${Open3D_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

#-------------------------------------------------------------------------------
# Library Linking and Dependency Resolution
#-------------------------------------------------------------------------------
target_link_libraries(stitch_node
    Eigen3::Eigen
    ${Open3D_LIBRARIES}
    CUDA::cudart
    CUDA::cublas
    pthread
)

ament_target_dependencies(stitch_node
    rclcpp sensor_msgs geometry_msgs std_msgs tf2_ros rosbag2_cpp
)

#-------------------------------------------------------------------------------
# Installation Configuration for ROS 2 Workspace
#-------------------------------------------------------------------------------
install(TARGETS stitch_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include/)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)
install(DIRECTORY launch/  DESTINATION share/${PROJECT_NAME}/launch)

#-------------------------------------------------------------------------------
# Finalize ROS 2 Package Configuration
#-------------------------------------------------------------------------------
ament_package()
