/**
 * @file temporal_persistence_filter.cpp
 * @brief Temporal persistence filtering for point cloud stabilization
 * 
 * Implements a voxel-based temporal persistence filter that identifies and retains
 * consistently observed points across multiple frames. This reduces noise and
 * improves the stability of the point cloud reconstruction.
 */
#include "doppler_icp_stitcher_open3d_pro2/temporal_persistence_filter.h"
#include <cmath>
// ================= Private Method Implementations =================
/**
 * @brief Quantizes 3D point coordinates to create spatial hash keys
 * 
 * Converts continuous 3D coordinates into discrete voxel indices for efficient
 * spatial hashing and persistence tracking. This enables O(1) lookups for point existence.
 * 
 * @param pt 3D point coordinates to quantize
 * @return std::string Unique string key representing the quantized voxel position
 * 
 * @note Voxel size determines the spatial resolution of persistence tracking
 *       Smaller voxel sizes provide higher precision but require more memory
 */
std::string TemporalPersistenceFilter::quantize_point_key(const Eigen::Vector3d& pt) {
    // Quantize coordinates by dividing by voxel size and rounding to nearest integer
    int x = std::round(pt.x() / voxel_size_);
    int y = std::round(pt.y() / voxel_size_);
    int z = std::round(pt.z() / voxel_size_);
    // Create unique string key from quantized coordinates
    return std::to_string(x) + "_" + std::to_string(y) + "_" + std::to_string(z);
}
/**
 * @brief Creates filtered point cloud containing only persistent points
 * 
 * Generates a new point cloud by filtering out non-persistent points that haven't
 * been observed for sufficient consecutive frames. This significantly reduces
 * transient noise and artifacts.
 * 
 * @param original Input point cloud data to filter
 * @return PointCloudData Output containing only highly persistent points
 * 
 * @note Maintains all original attributes: points, velocities, normals, and timestamps
 * @warning Point cloud data structures are assumed to be properly aligned and sized
 */
PointCloudData TemporalPersistenceFilter::create_persistent_pointcloud(const PointCloudData& original) {
    PointCloudData filtered;
    std::vector<int> persistent_indices;
   
     // First pass: identify indices of points that meet persistence threshold
    for (int i = 0; i < original.points.rows(); ++i) {
        Eigen::Vector3d pt = original.points.row(i);
        std::string key = quantize_point_key(pt);
        // Check if point exists in persistence map and meets minimum persistence count
        auto it = persistent_points_.find(key);
        if (it != persistent_points_.end() && it->second.persistence_count >= min_persistence_) {
            persistent_indices.push_back(i); // Point is persistent, include in output
        }
    }
    // Allocate output arrays based on number of persistent points
    filtered.points.resize(persistent_indices.size(), 3);
    filtered.velocities.resize(persistent_indices.size());
    filtered.normals.resize(persistent_indices.size(), 3);
    // Copy persistent points and their attributes to output
    for (size_t j = 0; j < persistent_indices.size(); ++j) {
        int idx = persistent_indices[j];
        filtered.points.row(j) = original.points.row(idx);
        filtered.velocities(j) = original.velocities(idx);
        // Copy normals if they exist in original data
        if (original.normals.rows() > 0) {
            filtered.normals.row(j) = original.normals.row(idx);
        }
    }
   
    // Preserve timestamp information for temporal consistency
    filtered.frame_timestamp_seconds = original.frame_timestamp_seconds;
    filtered.frame_timestamp_nanoseconds = original.frame_timestamp_nanoseconds;
    
    // Log filtering statistics for monitoring and debugging
    RCLCPP_INFO(rclcpp::get_logger("persistence_filter"),
               "Persistent points: %zu/%ld (%.1f%%)",
               persistent_indices.size(), original.points.rows(),
               (100.0 * persistent_indices.size()) / original.points.rows());
   
    return filtered;
}

// ================= Public Method Implementations =================

/**
 * @brief Main filtering interface - processes new frame and returns filtered result
 * 
 * Updates persistence tracking with new frame data and generates filtered output
 * containing only points that demonstrate consistent temporal behavior.
 * 
 * @param new_frame Incoming point cloud frame to process
 * @return PointCloudData Filtered point cloud with persistent points only
 * 
 * @note This method maintains internal state across multiple calls
 * @warning Thread safety must be considered if used in multi-threaded environments
 */
PointCloudData TemporalPersistenceFilter::filter_non_persistent_points(const PointCloudData& new_frame) {
    current_frame_++; // Increment frame counter for temporal tracking
    std::map<std::string, bool> current_frame_points;
   
    // Phase 1: Quantize all points in current frame for spatial hashing
    for (int i = 0; i < new_frame.points.rows(); ++i) {
        Eigen::Vector3d pt = new_frame.points.row(i);
        std::string key = quantize_point_key(pt);
        current_frame_points[key] = true;   // Mark point as present in current frame
    }
   
    // Phase 2: Update persistence counts for existing tracked points
    std::vector<std::string> to_remove;
    for (auto& [key, persistent_pt] : persistent_points_) {
        if (current_frame_points.find(key) != current_frame_points.end()) {
             // Point still exists in current frame - reinforce persistence
            persistent_pt.persistence_count++;
            persistent_pt.last_seen_frame = current_frame_;
        } else {
            // Point disappeared - consider for removal if absent for multiple frames
            if (current_frame_ - persistent_pt.last_seen_frame > 2) {
                to_remove.push_back(key);
            }
        }
    }
   
    // Phase 3: Clean up points that have been absent for too long
    for (const auto& key : to_remove) {
        persistent_points_.erase(key);
    }
   
    // Phase 4: Initialize tracking for new points in current frame
    for (int i = 0; i < new_frame.points.rows(); ++i) {
        Eigen::Vector3d pt = new_frame.points.row(i);
        std::string key = quantize_point_key(pt);
        // Only add if not already being tracked (avoid duplicates)
        if (persistent_points_.find(key) == persistent_points_.end()) {
            persistent_points_[key] = {pt, 1, current_frame_};
        }
    }
   
    // Phase 5: Generate final filtered output
    return create_persistent_pointcloud(new_frame);
}
/**
 * @brief Configures filter parameters for adaptive behavior
 * 
 * Allows runtime adjustment of spatial resolution and persistence requirements
 * to adapt to different environmental conditions and performance needs.
 * 
 * @param voxel_size Spatial resolution for point quantization (meters)
 * @param min_persistence Minimum consecutive frames for point to be considered persistent
 * 
 * @note Smaller voxel sizes increase memory usage but improve spatial precision
 * @warning Changing parameters resets internal state - consider calling before processing
 */
void TemporalPersistenceFilter::set_parameters(double voxel_size, int min_persistence) {
    voxel_size_ = voxel_size;
    min_persistence_ = min_persistence;
    
}
