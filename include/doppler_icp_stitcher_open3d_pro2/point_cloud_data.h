/**
 * @file point_cloud_data.h
 * @brief Point cloud data structure with comprehensive metadata
 * 
 * Defines the core data structure for storing point clouds with geometric
 * properties, temporal information, and precomputed quantities for optimization.
 */
#ifndef POINT_CLOUD_DATA_H
#define POINT_CLOUD_DATA_H

#include <Eigen/Dense>

// ================= Point Cloud Data =================
/**
 * @brief Comprehensive point cloud container with geometric and temporal attributes
 * 
 * Stores 3D point clouds with associated velocities, normals, and timestamps.
 * Includes precomputed quantities for efficient algorithm execution.
 */
struct PointCloudData {
    /// 3D point coordinates (N x 3 matrix, each row is [x, y, z])
    Eigen::MatrixXd points;
    /// Radial velocity measurements for each point (N-dimensional vector)
    Eigen::VectorXd velocities;
    /// Surface normal vectors at each point (N x 3 matrix)
    Eigen::MatrixXd normals;
   
    // ================= Temporal Metadata =================
    
    /// Frame timestamp seconds component (from CSV data)
    double frame_timestamp_seconds = 0.0;
    /// Frame timestamp nanoseconds component (for high-precision timing)
    double frame_timestamp_nanoseconds = 0.0;
   
    // ================= Precomputed Quantities =================
    
    /// Euclidean norms of each point vector (cached for performance)
    Eigen::VectorXd point_norms;
    /// Unit direction vectors for each point (points normalized to unit length)
    Eigen::MatrixXd unit_directions;
   /**
     * @brief Precomputes derived quantities for algorithm optimization
     * 
     * Computes point norms and unit directions to avoid redundant calculations
     * during ICP registration and Doppler constraint evaluation.
     * 
     * @note Called automatically after point cloud loading/preprocessing
     * @warning Should be called whenever points array is modified
     */
    void precompute() {
        if (points.rows() == 0) return;
        // Compute Euclidean norms for all points
        point_norms = points.rowwise().norm();
        // Precompute unit direction vectors
        unit_directions.resize(points.rows(), 3);
        for (int i = 0; i < points.rows(); ++i) {
            double norm = point_norms(i);
            if (norm < 1e-12) norm = 1.0; // Avoid division by zero
            unit_directions.row(i) = points.row(i) / norm;
        }
    }
};

#endif // POINT_CLOUD_DATA_H
