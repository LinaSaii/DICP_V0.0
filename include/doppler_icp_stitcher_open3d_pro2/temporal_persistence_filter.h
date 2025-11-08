/**
 * @file temporal_persistence_filter.h
 * @brief Temporal persistence filtering for point cloud stabilization
 * 
 * Implements voxel-based temporal filtering to identify and retain consistently
 * observed points across multiple frames, reducing noise and improving
 * reconstruction stability.
 */
#ifndef TEMPORAL_PERSISTENCE_FILTER_H
#define TEMPORAL_PERSISTENCE_FILTER_H

#include "point_cloud_data.h"
#include <Eigen/Dense>
#include <map>
#include <string>
#include <vector>
#include <rclcpp/rclcpp.hpp>

// ================= Temporal Persistence Filter Class =================

/**
 * @brief Implements temporal persistence filtering for point cloud denoising
 * 
 * Tracks points across multiple frames using voxel-based spatial hashing and
 * retains only those points that demonstrate consistent temporal presence.
 * This effectively filters out transient noise and moving objects.
 */
class TemporalPersistenceFilter {
private:
    /**
     * @brief Internal structure for tracking persistent point statistics
     */
    struct PersistentPoint {
        Eigen::Vector3d position;    ///< 3D position of the tracked point
        int persistence_count;       ///< Number of consecutive frames observed
        int last_seen_frame;         ///< Most recent frame where point was observed
    };
    /// Spatial hash map for efficient point tracking (key: quantized position)
    std::map<std::string, PersistentPoint> persistent_points_;
    /// Current frame counter for temporal tracking
    int current_frame_ = 0;
    /// Voxel size for spatial quantization (meters)
    double voxel_size_ = 0.1; // 10cm voxel grid
    /// Minimum consecutive frames for point to be considered persistent
    int min_persistence_ = 5; // Must appear in 5 consecutive frames
    /**
     * @brief Quantizes 3D point coordinates to create spatial hash keys
     * @param pt 3D point coordinates to quantize
     * @return std::string Unique string key representing quantized voxel
     * @note Implementation in temporal_persistence_filter.cpp
     */
    std::string quantize_point_key(const Eigen::Vector3d& pt);
    /**
     * @brief Creates filtered point cloud containing only persistent points
     * @param original Input point cloud to filter
     * @return PointCloudData Output with persistent points only
     * @note Implementation in temporal_persistence_filter.cpp
     */
    PointCloudData create_persistent_pointcloud(const PointCloudData& original);

public:
    /**
     * @brief Main filtering interface - processes new frame and returns filtered result
     * 
     * Updates persistence tracking with new frame data and generates output
     * containing only points that meet the minimum persistence threshold.
     * 
     * @param new_frame Incoming point cloud frame to process
     * @return PointCloudData Filtered point cloud with persistent points only
     */
    PointCloudData filter_non_persistent_points(const PointCloudData& new_frame);
    /**
     * @brief Configures filter parameters for adaptive behavior
     * 
     * @param voxel_size Spatial resolution for point quantization (meters)
     * @param min_persistence Minimum consecutive frames for point persistence
     */
    void set_parameters(double voxel_size, int min_persistence);
};

#endif // TEMPORAL_PERSISTENCE_FILTER_H
