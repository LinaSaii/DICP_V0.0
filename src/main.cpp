/**
 * @file main.cpp
 * @brief Main entry point for Doppler ICP Stitcher ROS 2 node
 * 
 * Initializes the ROS 2 framework and launches the DopplerICPStitcher node.
 * This serves as the primary executable for the point cloud stitching pipeline.
 */

#include <rclcpp/rclcpp.hpp>
#include "doppler_icp_stitcher_open3d_pro2/doppler_icp_stitcher.h"
// ================= Main Application Entry Point =================

/**
 * @brief Main function - initializes ROS 2 and starts the Doppler ICP Stitcher node
 * 
 * This function performs the following key operations:
 * 1. Initializes the ROS 2 C++ client library
 * 2. Creates and configures the DopplerICPStitcher node
 * 3. Enters the main processing loop (spin)
 * 4. Handles graceful shutdown upon termination
 * 
 * @param argc Command line argument count
 * @param argv Command line argument vector
 * @return int Exit status code (0 for normal termination)
 * 
 * @note Exception safety: All exceptions are caught by ROS 2 infrastructure
 * @warning Ensure proper ROS 2 environment setup before execution
 */
// ================= Main =================
int main(int argc, char** argv) {
    // Phase 1: ROS 2 Framework Initialization
    rclcpp::init(argc, argv);
    // Phase 2: Node Creation and Configuration
    auto node = std::make_shared<DopplerICPStitcher>();
    // Phase 3: Main Processing Loop
    // This call blocks until shutdown signal is received (Ctrl+C)
    rclcpp::spin(node);
    // Phase 4: Graceful Shutdown
    rclcpp::shutdown();
    // Successful termination
    return 0;
}
