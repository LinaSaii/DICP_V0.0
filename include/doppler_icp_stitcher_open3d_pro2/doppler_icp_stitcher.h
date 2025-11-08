/**
 * @file doppler_icp_stitcher.h
 * @brief Main Doppler ICP Stitcher ROS 2 node declaration
 * 
 * Defines the core ROS 2 node implementing the complete Doppler-enhanced ICP
 * pipeline with multi-parameter support, temporal filtering, and comprehensive
 * logging capabilities.
 */

#ifndef DOPPLER_ICP_STITCHER_H
#define DOPPLER_ICP_STITCHER_H

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <open3d/Open3D.h>
#include <Eigen/Dense>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <iostream>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <future>
#include <atomic>
#include <queue>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <functional>
#include <ctime>
#include <map>
#include <set>
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include "point_cloud_data.h"
#include "temporal_persistence_filter.h"
#include "async_preprocessor.h"
#include "utils.h"

using namespace std::chrono_literals;
namespace fs = std::filesystem;

// ================= Doppler ICP Stitcher ROS 2 Node =================

/**
 * @brief Main ROS 2 node implementing Doppler-enhanced ICP point cloud stitching
 * 
 * Features comprehensive point cloud processing pipeline with:
 * - Multi-parameter set execution for systematic evaluation
 * - Temporal persistence filtering for noise reduction
 * - Asynchronous preprocessing for performance optimization
 * - MCAP recording for data persistence
 * - Adaptive normal estimation based on scene characteristics
 */
class DopplerICPStitcher : public rclcpp::Node {
public:
    /**
     * @brief Constructs and initializes the Doppler ICP Stitcher node
     */
    DopplerICPStitcher();
    
    /**
     * @brief Destructor - ensures graceful resource cleanup
     */
    ~DopplerICPStitcher();

private:
    // ================= Internal Data Structures =================
    
    /**
     * @brief Complete parameter set for ICP algorithm configuration
     * 
     * Groups all tunable parameters for systematic batch execution
     * and comparative analysis across parameter combinations.
     */
    struct ParameterSet {
        // Temporal persistence filter parameters
        bool enable_persistence_filter;    ///< Enable/disable persistence filtering
        double persistence_voxel_size;     ///< Voxel size for spatial quantization
        int min_persistence_frames;        ///< Minimum frames for point persistence
       
        // Core ICP algorithm parameters
        double velocity_threshold;         ///< Radial velocity filtering threshold
        int downsample_factor;             ///< Uniform downsampling factor
        int max_iterations;                ///< Maximum DICP iterations
        double icp_tolerance;              ///< Convergence tolerance
        double publish_rate;               ///< Frame processing rate (Hz)
        double lambda_doppler_start;       ///< Initial Doppler weight
        double lambda_doppler_end;         ///< Final Doppler weight
        int lambda_schedule_iters;         ///< Doppler weight scheduling iterations
        double frame_dt;                   ///< Inter-frame time delta
        double t_vl_x;                     ///< X-component of sensor velocity
        double t_vl_y;                     ///< Y-component of sensor velocity
        double t_vl_z;                     ///< Z-component of sensor velocity
        bool reject_outliers;              ///< Enable outlier rejection
        double outlier_thresh;             ///< Outlier velocity threshold
        int rejection_min_iters;           ///< Minimum iterations before outlier rejection
        int geometric_min_iters;           ///< Minimum iterations for geometric weights
        int doppler_min_iters;             ///< Minimum iterations for Doppler weights
        double geometric_k;                ///< Huber threshold for geometric residuals
        double doppler_k;                  ///< Huber threshold for Doppler residuals
        double max_corr_distance;          ///< Maximum correspondence distance
        int min_inliers;                   ///< Minimum inliers for valid registration
        int last_n_frames;                 ///< Sliding window size for stitching
        bool use_voxel_filter;             ///< Enable voxel downsampling
        double voxel_size;                 ///< Voxel size for downsampling
       
        // Adaptive normal estimation parameters
        std::string normal_estimation_mode; ///< "auto", "vertical", or "estimated"
        double static_scene_threshold;      ///< Z-variance threshold for static scenes
    };
    
    /**
     * @brief Frame processing statistics for performance monitoring
     */
    struct FrameStats {
        size_t frame_index;                ///< Sequential frame identifier
        size_t initial_points;             ///< Points before any filtering
        size_t filtered_points;            ///< Points after velocity filtering
        int iterations_used;               ///< ICP iterations actually used
        double processing_time_ms;         ///< Total processing time (milliseconds)
        std::string filename;              ///< Source filename for tracking
        size_t parameter_set_index;        ///< Active parameter set index
    };
    
    // ================= Timing Control Members =================
    
    std::chrono::steady_clock::time_point last_frame_time_;  ///< Last frame processing time
    bool first_frame_processed_;                             ///< First frame completion flag
    const double target_frame_time_;                         ///< Target inter-frame interval
    
    // ================= MCAP Recording System =================
    
    std::unique_ptr<rosbag2_cpp::Writer> bag_writer_;  ///< MCAP bag writer instance
    bool enable_recording_;                            ///< Recording enable/disable flag
    std::string recording_directory_;                  ///< Recording output directory
    std::string recording_filename_;                   ///< Generated recording filename
    
    // ================= Temporal Persistence Filter =================
    
    TemporalPersistenceFilter persistence_filter_;     ///< Persistence filter instance
    bool enable_persistence_filter_;                   ///< Filter enable/disable
    double persistence_voxel_size_;                    ///< Filter voxel size
    int min_persistence_frames_;                       ///< Minimum persistence frames
    
    // ================= Pipeline Statistics Tracking =================
    
    size_t current_frame_points_after_persistence_ = 0; ///< Points count after persistence filter
    
    // ================= Private Method Declarations =================
    
    /**
     * @brief Initializes MCAP recording system
     * @note Creates directory and configures bag writer
     */
    void initialize_recording();
    
    /**
     * @brief Records ROS 2 message to MCAP bag file
     * @tparam T ROS 2 message type
     * @param message Message to record
     * @param topic_name Topic identifier for message
     */
    template<typename T>
    void record_message(const T& message, const std::string& topic_name);
    
    /**
     * @brief Generates all parameter combinations from ROS parameters
     * @note Creates Cartesian product of all parameter arrays
     */
    void initialize_parameter_combinations();
    
    /**
     * @brief Activates specific parameter set for current processing
     * @param param_index Index of parameter set to activate
     */
    void set_current_parameters(size_t param_index);
    
    /**
     * @brief Initializes enhanced CSV file for trajectory logging
     * @note Includes all parameters for full experiment reproducibility
     */
    void initialize_csv_file();
    
    /**
     * @brief Initializes execution logs for performance monitoring
     */
    void initialize_logs_file();
    
    /**
     * @brief Saves frame statistics to execution logs
     * @param stats Frame statistics to record
     */
    void save_frame_stats_to_logs(const FrameStats& stats);
    
    /**
     * @brief Saves pose and comprehensive metadata to CSV
     * @param frame_idx Frame index for temporal tracking
     * @param pose Estimated transformation matrix
     * @param frame_data Source point cloud data
     */
    void save_pose_to_csv(size_t frame_idx, const Eigen::Matrix4d& pose, const PointCloudData& frame_data);
    
    /**
     * @brief Classifies scene as static or dynamic based on geometry
     * @param points Point cloud coordinates matrix
     * @return bool True if scene is static, false if dynamic
     */
    bool is_static_scene(const Eigen::MatrixXd& points);
    
    // ================= Asynchronous Pipeline Methods =================
    
    void start_async_preprocessing(size_t frame_idx);
    PointCloudData get_next_preprocessed_frame();
    
    // ================= Core Processing Methods =================
    
    PointCloudData load_frame(const std::string& filename);
    PointCloudData preprocess_point_cloud(const PointCloudData& input);
    inline Eigen::VectorXd huber_weights(const Eigen::VectorXd& residuals, double k);
    std::pair<Eigen::Matrix4d, double> doppler_icp(const PointCloudData& source, const PointCloudData& target);
    void process_next_frame();
    
    // ================= Publishing Methods =================
    
    void publish_pointcloud();
    void publish_current_pose();
    void publish_trajectory();
    void publish_lin_acc_ang_vel(const Eigen::Vector3d& lin_acc, const Eigen::Vector3d& ang_vel);
    void publish_tf();
    
    // ================= Member Variables =================
    
    // Core data paths
    std::string frames_dir_;
    std::string output_csv_path_;
   
    // Current active parameter values
    double velocity_threshold_;
    int downsample_factor_;
    int max_iterations_;
    double icp_tolerance_;
    double publish_rate_;
    double lambda_doppler_start_;
    double lambda_doppler_end_;
    int lambda_schedule_iters_;
    double frame_dt_;
    double t_vl_x_;
    double t_vl_y_;
    double t_vl_z_;
    bool reject_outliers_;
    double outlier_thresh_;
    int rejection_min_iters_;
    int geometric_min_iters_;
    int doppler_min_iters_;
    double geometric_k_;
    double doppler_k_;
    double max_corr_distance_;
    int min_inliers_;
    int last_n_frames_;
    bool use_voxel_filter_;
    double voxel_size_;
   
    // Adaptive normal estimation
    std::string normal_estimation_mode_;
    double static_scene_threshold_;
    std::string current_normal_mode_ = "estimated";
    
    // Parameter combinations and state
    std::vector<ParameterSet> parameter_sets_;
    size_t current_param_index_ = 0;
    
    // Frame statistics
    size_t current_frame_initial_points_ = 0;
    size_t current_frame_filtered_points_ = 0;
    int current_frame_iterations_ = 0;
    
    // Processing state
    size_t frame_idx_;
    std::vector<std::string> frame_files_;
    std::vector<std::pair<PointCloudData, Eigen::Matrix4d>> stitched_frames_;
    std::vector<Eigen::Matrix4d> trajectory_;
   
    PointCloudData previous_frame_;
    bool previous_frame_set_;
    Eigen::Matrix4d current_pose_;
    
    // File I/O
    std::ofstream csv_file_;
    std::string icp_pose_dir_;
    std::string excel_filename_;
    std::ofstream logs_file_;
    std::string logs_dir_;
    std::string logs_filename_;
    
    // ================= Real-time Pipeline Members =================
    
    std::unique_ptr<AsyncPreprocessor> preprocessor_;
    std::future<PointCloudData> next_frame_future_;
    std::atomic<bool> preprocessing_next_frame_{false};
    size_t next_frame_idx_{1};
    
    // ROS 2 Publishers
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr trajectory_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr lin_acc_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr ang_vel_pub_;
   
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;
};

#endif // DOPPLER_ICP_STITCHER_H