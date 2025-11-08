/**
 * @brief Initializes enhanced CSV file for comprehensive trajectory logging
 * 
 * Creates timestamped CSV file with complete parameter metadata for each pose.
 * This enables post-processing analysis and correlation between parameters and performance.
 * 
 * @note File includes all active parameters for full experiment reproducibility
 * @warning File creation failures are logged but non-fatal
 */
#include "doppler_icp_stitcher_open3d_pro2/doppler_icp_stitcher.h"

// ================= Constructor and Destructor =================

/**
 * @brief Constructs and initializes the Doppler ICP Stitcher node
 * 
 * Performs comprehensive initialization including:
 * - ROS 2 parameter declaration and retrieval
 * - MCAP recording system setup
 * - Persistence filter configuration
 * - Multi-parameter combination generation
 * - Publisher and subscriber initialization
 * - Asynchronous preprocessing pipeline startup
 * 
 * @note Exception safe - failures are logged but don't terminate construction
 * @warning Heavy initialization - consider performance implications for real-time use
 */

DopplerICPStitcher::DopplerICPStitcher() : Node("doppler_icp_stitcher"),
                          preprocessor_(std::make_unique<AsyncPreprocessor>()),
                          target_frame_time_(0.0),
                          enable_recording_(false) {

    // ================= Phase 1: Parameter Declaration =================
    // Core data path parameters
    declare_parameter("frames_directory", "/home/zassi/test_lidar/data/outdoor/outdoor_stationary_scene/outdoor_stationary_scene/stationary_six_column_csv");
    declare_parameter("output_csv_path", "/home/zassi/ros2_ws/icp_pose/outdoor.csv");
   
    // MCAP recording parameters
    declare_parameter("enable_recording", true);
    declare_parameter("recording_directory", "./recordings");
   
    // Temporal persistence filter parameters (array for multiple configurations)
    declare_parameter("enable_persistence_filter", std::vector<bool>{true});
    declare_parameter("persistence_voxel_size", std::vector<double>{1.0});
    declare_parameter("min_persistence_frames", std::vector<int>{10});
   
    // DICP algorithm parameter arrays for batch execution
    declare_parameter("velocity_threshold", std::vector<double>{0.1});
    declare_parameter("downsample_factor", std::vector<int>{1});
    declare_parameter("max_iterations", std::vector<int>{10});
    declare_parameter("icp_tolerance", std::vector<double>{1e-9});
    declare_parameter("publish_rate", std::vector<double>{10.0});
    declare_parameter("lambda_doppler_start", std::vector<double>{0.0});
    declare_parameter("lambda_doppler_end", std::vector<double>{0.0});
    declare_parameter("lambda_schedule_iters", std::vector<int>{0});
    declare_parameter("frame_dt", std::vector<double>{0.1});
    declare_parameter("t_vl_x", std::vector<double>{0.0});
    declare_parameter("t_vl_y", std::vector<double>{0.0});
    declare_parameter("t_vl_z", std::vector<double>{0.0});
    declare_parameter("reject_outliers", std::vector<bool>{true});
    declare_parameter("outlier_thresh", std::vector<double>{0.1});
    declare_parameter("rejection_min_iters", std::vector<int>{2});
    declare_parameter("geometric_min_iters", std::vector<int>{0});
    declare_parameter("doppler_min_iters", std::vector<int>{2});
    declare_parameter("geometric_k", std::vector<double>{0.2});
    declare_parameter("doppler_k", std::vector<double>{0.3});
    declare_parameter("max_corr_distance", std::vector<double>{0.1});
    declare_parameter("min_inliers", std::vector<int>{5});
    declare_parameter("last_n_frames", std::vector<int>{15});
    declare_parameter("use_voxel_filter", std::vector<bool>{false});
    declare_parameter("voxel_size", std::vector<double>{0.0001});
   
    // Adaptive normal estimation parameters for scene-aware processing
    declare_parameter("normal_estimation_mode", std::vector<std::string>{"vertical"}); // "auto", "vertical", "estimated"
    declare_parameter("static_scene_threshold", std::vector<double>{0.5}); // Z-variance threshold for static scene detection

    // ================= Phase 2: Parameter Retrieval =================
    frames_dir_ = get_parameter("frames_directory").as_string();
    output_csv_path_ = get_parameter("output_csv_path").as_string();
   
    // MCAP recording initialization
    enable_recording_ = get_parameter("enable_recording").as_bool();
    recording_directory_ = get_parameter("recording_directory").as_string();
   
    if (enable_recording_) {
        initialize_recording();
    }
   
     // ================= Phase 3: Multi-Parameter Setup =================
    
    // Generate all parameter combinations for batch processing
    initialize_parameter_combinations();
   
    // Activate first parameter set (index 0)
    set_current_parameters(0);

    // ================= Phase 4: Logging System Initialization =================
    
    // Enhanced CSV file for trajectory and parameter logging
    initialize_csv_file();
    // Execution logs for performance and debugging
    initialize_logs_file();

    // ================= Phase 5: Data Loading and Preparation =================
    
    // Load and sort CSV frame files using natural sorting
    for (auto& entry : fs::directory_iterator(frames_dir_)) {
        if (entry.path().extension() == ".csv")
            frame_files_.push_back(entry.path().string());
    }
    std::sort(frame_files_.begin(), frame_files_.end(), natural_sort);
    RCLCPP_INFO(get_logger(), "Loaded %zu CSV frames from %s", frame_files_.size(), frames_dir_.c_str());

     // ================= Phase 6: ROS 2 Communication Setup =================
    
    // Quality of Service configuration for optimal performance
    auto qos = rclcpp::QoS(rclcpp::KeepLast(1));
    qos.best_effort();
    // Publisher initialization for all output topics
    pointcloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("stitched_cloud", qos);
    pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("icp_pose", qos);
    trajectory_pub_ = create_publisher<geometry_msgs::msg::PoseArray>("icp_trajectory", qos);
    lin_acc_pub_ = create_publisher<geometry_msgs::msg::Vector3Stamped>("linear_acceleration", qos);
    ang_vel_pub_ = create_publisher<geometry_msgs::msg::Vector3Stamped>("angular_velocity", qos);
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    // ================= Phase 7: State Variable Initialization =================
    current_pose_ = Eigen::Matrix4d::Identity();
    frame_idx_ = 0;
    next_frame_idx_ = 1;
    previous_frame_set_ = false;
    preprocessing_next_frame_.store(false);
    // Timing control for consistent frame rate processing
    last_frame_time_ = std::chrono::steady_clock::now();
    first_frame_processed_ = false;

    // ================= Phase 8: Pipeline Startup =================
    
    // Start asynchronous preprocessing of next frame if available
    if (frame_files_.size() > 1) {
        start_async_preprocessing(next_frame_idx_);
    }

    // ================= Phase 9: Main Processing Timer =================
    
    // Create wall timer for frame processing at specified publish rate
    timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / publish_rate_),
        std::bind(&DopplerICPStitcher::process_next_frame, this));
}

/**
 * @brief Destructor - ensures graceful cleanup of all resources
 * 
 * Performs orderly shutdown including:
 * - CSV file closure and finalization
 * - Execution logs file closure
 * - MCAP recording termination
 * - Asynchronous preprocessing thread termination (handled by AsyncPreprocessor destructor)
 * 
 * @note All file operations include error checking and logging
 * @warning Destructor should not throw exceptions
 */
DopplerICPStitcher::~DopplerICPStitcher() {
    // Close and finalize enhanced trajectory CSV file
    if (csv_file_.is_open()) {
        csv_file_.close();
        RCLCPP_INFO(get_logger(), "Enhanced trajectory saved to: %s", excel_filename_.c_str());
    }
   
    // Close execution logs file
    if (logs_file_.is_open()) {
        logs_file_.close();
        RCLCPP_INFO(get_logger(), "Execution logs saved to: %s", logs_filename_.c_str());
    }
   
    // Close MCAP recording with proper termination
    if (bag_writer_) {
        bag_writer_->close();
        RCLCPP_INFO(get_logger(), "MCAP recording saved: %s/%s",
                   recording_directory_.c_str(), recording_filename_.c_str());
    }
}

/**
 * @brief Initializes MCAP recording system for data persistence
 * 
 * Creates recording directory and configures MCAP writer with timestamped filename.
 * MCAP format provides efficient ROS 2 bag recording with better performance than legacy formats.
 * 
 * @note MCAP recording requires rosbag2_cpp dependencies
 * @warning Recording failures are non-fatal - system continues without recording
 * @exception std::exception Captures and logs any MCAP initialization errors
 */

void DopplerICPStitcher::initialize_recording() {
    // Create recording directory structure
    std::filesystem::create_directories(recording_directory_);
   
    // Generate timestamped filename for unique identification
    auto now = std::chrono::system_clock::now();
    auto now_time_t = std::chrono::system_clock::to_time_t(now);
    std::tm now_tm = *std::localtime(&now_time_t);
   
    std::stringstream filename;
    filename << "doppler_icp_"
             << std::put_time(&now_tm, "%Y%m%d_%H%M%S")
             << ".mcap";
   
    recording_filename_ = filename.str();
    std::string full_path = recording_directory_ + "/" + recording_filename_;
   
    try {
        bag_writer_ = std::make_unique<rosbag2_cpp::Writer>();
       
        rosbag2_storage::StorageOptions storage_options;
        storage_options.uri = full_path;
        storage_options.storage_id = "mcap";
       
        // Use default converter options for optimal performance
        bag_writer_->open(storage_options);
       
        RCLCPP_INFO(get_logger(), "Started MCAP recording: %s", full_path.c_str());
       
    } catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "Failed to initialize MCAP recording: %s", e.what());
        RCLCPP_WARN(get_logger(), "Continuing without MCAP recording");
        enable_recording_ = false;
        bag_writer_.reset();
    }
}
/**
 * @brief Template method for recording any ROS 2 message to MCAP
 * 
 * Provides type-safe message recording with comprehensive error handling.
 * Used for persisting all published topics for later analysis and debugging.
 * 
 * @tparam T ROS 2 message type (auto-deduced)
 * @param message ROS 2 message to record
 * @param topic_name Topic identifier for message categorization
 * 
 * @note Silent failure - recording errors don't affect main processing pipeline
 * @warning Topic name must match actual publication topic for proper playback
 */
template<typename T>
void DopplerICPStitcher::record_message(const T& message, const std::string& topic_name) {
    if (!enable_recording_ || !bag_writer_) {
        return;
    }
   
    try {
        bag_writer_->write(message, topic_name, now());
    } catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "Failed to record message on topic %s: %s",
                    topic_name.c_str(), e.what());
    }
}
/**
 * @brief Generates all parameter combinations for batch processing
 * 
 * Creates Cartesian product of all parameter arrays to enable systematic
 * parameter sweeps and comparative analysis. Handles arrays of different lengths
 * by using first element as default for shorter arrays.
 * 
 * @note Parameter combinations are stored in parameter_sets_ vector
 * @warning Large parameter spaces can cause combinatorial explosion
 * @complexity O(n) where n is the longest parameter array
 */
void DopplerICPStitcher::initialize_parameter_combinations() {
    // Retrieve all parameter arrays from ROS 2 parameter server
    auto enable_persistence_filters = get_parameter("enable_persistence_filter").as_bool_array();
    auto persistence_voxel_sizes = get_parameter("persistence_voxel_size").as_double_array();
    auto min_persistence_frames_list = get_parameter("min_persistence_frames").as_integer_array();
   
    auto velocity_thresholds = get_parameter("velocity_threshold").as_double_array();
    auto downsample_factors = get_parameter("downsample_factor").as_integer_array();
    auto max_iterations_list = get_parameter("max_iterations").as_integer_array();
    auto icp_tolerances = get_parameter("icp_tolerance").as_double_array();
    auto publish_rates = get_parameter("publish_rate").as_double_array();
    auto lambda_doppler_starts = get_parameter("lambda_doppler_start").as_double_array();
    auto lambda_doppler_ends = get_parameter("lambda_doppler_end").as_double_array();
    auto lambda_schedule_iters_list = get_parameter("lambda_schedule_iters").as_integer_array();
    auto frame_dts = get_parameter("frame_dt").as_double_array();
    auto t_vl_x_list = get_parameter("t_vl_x").as_double_array();
    auto t_vl_y_list = get_parameter("t_vl_y").as_double_array();
    auto t_vl_z_list = get_parameter("t_vl_z").as_double_array();
    auto reject_outliers_list = get_parameter("reject_outliers").as_bool_array();
    auto outlier_thresh_list = get_parameter("outlier_thresh").as_double_array();
    auto rejection_min_iters_list = get_parameter("rejection_min_iters").as_integer_array();
    auto geometric_min_iters_list = get_parameter("geometric_min_iters").as_integer_array();
    auto doppler_min_iters_list = get_parameter("doppler_min_iters").as_integer_array();
    auto geometric_k_list = get_parameter("geometric_k").as_double_array();
    auto doppler_k_list = get_parameter("doppler_k").as_double_array();
    auto max_corr_distances = get_parameter("max_corr_distance").as_double_array();
    auto min_inliers_list = get_parameter("min_inliers").as_integer_array();
    auto last_n_frames_list = get_parameter("last_n_frames").as_integer_array();
    auto use_voxel_filter_list = get_parameter("use_voxel_filter").as_bool_array();
    auto voxel_sizes = get_parameter("voxel_size").as_double_array();
   
    // Adaptive normal estimation parameters
    auto normal_estimation_modes = get_parameter("normal_estimation_mode").as_string_array();
    auto static_scene_thresholds = get_parameter("static_scene_threshold").as_double_array();
    // Determine maximum parameter set size for combination generation
    size_t max_size = std::max({
        enable_persistence_filters.size(), persistence_voxel_sizes.size(), min_persistence_frames_list.size(),
        velocity_thresholds.size(), downsample_factors.size(), max_iterations_list.size(),
        icp_tolerances.size(), publish_rates.size(), lambda_doppler_starts.size(),
        lambda_doppler_ends.size(), lambda_schedule_iters_list.size(), frame_dts.size(),
        t_vl_x_list.size(), t_vl_y_list.size(), t_vl_z_list.size(), reject_outliers_list.size(),
        outlier_thresh_list.size(), rejection_min_iters_list.size(), geometric_min_iters_list.size(),
        doppler_min_iters_list.size(), geometric_k_list.size(), doppler_k_list.size(),
        max_corr_distances.size(), min_inliers_list.size(), last_n_frames_list.size(),
        use_voxel_filter_list.size(), voxel_sizes.size(),
        normal_estimation_modes.size(), static_scene_thresholds.size()
    });
    // Reserve memory for optimal performance during population
    parameter_sets_.reserve(max_size);
    // Generate parameter combinations using index-based approach
    for (size_t i = 0; i < max_size; ++i) {
        ParameterSet params;
       
        // Persistence filter parameters with bounds checking
        params.enable_persistence_filter = i < enable_persistence_filters.size() ? enable_persistence_filters[i] : enable_persistence_filters[0];
        params.persistence_voxel_size = i < persistence_voxel_sizes.size() ? persistence_voxel_sizes[i] : persistence_voxel_sizes[0];
        params.min_persistence_frames = i < min_persistence_frames_list.size() ? min_persistence_frames_list[i] : min_persistence_frames_list[0];
        // Core DICP algorithm parameters
        params.velocity_threshold = i < velocity_thresholds.size() ? velocity_thresholds[i] : velocity_thresholds[0];
        params.downsample_factor = i < downsample_factors.size() ? downsample_factors[i] : downsample_factors[0];
        params.max_iterations = i < max_iterations_list.size() ? max_iterations_list[i] : max_iterations_list[0];
        params.icp_tolerance = i < icp_tolerances.size() ? icp_tolerances[i] : icp_tolerances[0];
        params.publish_rate = i < publish_rates.size() ? publish_rates[i] : publish_rates[0];
        params.lambda_doppler_start = i < lambda_doppler_starts.size() ? lambda_doppler_starts[i] : lambda_doppler_starts[0];
        params.lambda_doppler_end = i < lambda_doppler_ends.size() ? lambda_doppler_ends[i] : lambda_doppler_ends[0];
        params.lambda_schedule_iters = i < lambda_schedule_iters_list.size() ? lambda_schedule_iters_list[i] : lambda_schedule_iters_list[0];
        params.frame_dt = i < frame_dts.size() ? frame_dts[i] : frame_dts[0];
        params.t_vl_x = i < t_vl_x_list.size() ? t_vl_x_list[i] : t_vl_x_list[0];
        params.t_vl_y = i < t_vl_y_list.size() ? t_vl_y_list[i] : t_vl_y_list[0];
        params.t_vl_z = i < t_vl_z_list.size() ? t_vl_z_list[i] : t_vl_z_list[0];
        params.reject_outliers = i < reject_outliers_list.size() ? reject_outliers_list[i] : reject_outliers_list[0];
        params.outlier_thresh = i < outlier_thresh_list.size() ? outlier_thresh_list[i] : outlier_thresh_list[0];
        params.rejection_min_iters = i < rejection_min_iters_list.size() ? rejection_min_iters_list[i] : rejection_min_iters_list[0];
        params.geometric_min_iters = i < geometric_min_iters_list.size() ? geometric_min_iters_list[i] : geometric_min_iters_list[0];
        params.doppler_min_iters = i < doppler_min_iters_list.size() ? doppler_min_iters_list[i] : doppler_min_iters_list[0];
        params.geometric_k = i < geometric_k_list.size() ? geometric_k_list[i] : geometric_k_list[0];
        params.doppler_k = i < doppler_k_list.size() ? doppler_k_list[i] : doppler_k_list[0];
        params.max_corr_distance = i < max_corr_distances.size() ? max_corr_distances[i] : max_corr_distances[0];
        params.min_inliers = i < min_inliers_list.size() ? min_inliers_list[i] : min_inliers_list[0];
        params.last_n_frames = i < last_n_frames_list.size() ? last_n_frames_list[i] : last_n_frames_list[0];
        params.use_voxel_filter = i < use_voxel_filter_list.size() ? use_voxel_filter_list[i] : use_voxel_filter_list[0];
        params.voxel_size = i < voxel_sizes.size() ? voxel_sizes[i] : voxel_sizes[0];
       
        // Adaptive normal estimation parameters
        params.normal_estimation_mode = i < normal_estimation_modes.size() ? normal_estimation_modes[i] : normal_estimation_modes[0];
        params.static_scene_threshold = i < static_scene_thresholds.size() ? static_scene_thresholds[i] : static_scene_thresholds[0];
        parameter_sets_.push_back(params);
    }
    RCLCPP_INFO(get_logger(), "Initialized %zu parameter combinations", parameter_sets_.size());
}

/**
 * @brief Activates specific parameter set for current processing session
 * 
 * Applies all parameters from specified parameter set to active algorithm configuration.
 * This enables runtime parameter switching for comparative analysis and optimization.
 * 
 * @param param_index Index of parameter set to activate (0-based)
 * 
 * @warning Index bounds are checked - invalid indices generate error logs
 * @note Persistence filter parameters are immediately applied to filter instance
 */
void DopplerICPStitcher::set_current_parameters(size_t param_index) {
    // Validate parameter index range
    if (param_index >= parameter_sets_.size()) {
        RCLCPP_ERROR(get_logger(), "Parameter index out of range: %zu", param_index);
        return;
    }
    const auto& params = parameter_sets_[param_index];
    current_param_index_ = param_index;
    // Apply persistence filter parameters
    enable_persistence_filter_ = params.enable_persistence_filter;
    persistence_voxel_size_ = params.persistence_voxel_size;
    min_persistence_frames_ = params.min_persistence_frames;
    persistence_filter_.set_parameters(persistence_voxel_size_, min_persistence_frames_);
    // Apply core DICP algorithm parameters
    velocity_threshold_ = params.velocity_threshold;
    downsample_factor_ = params.downsample_factor;
    max_iterations_ = params.max_iterations;
    icp_tolerance_ = params.icp_tolerance;
    publish_rate_ = params.publish_rate;
    lambda_doppler_start_ = params.lambda_doppler_start;
    lambda_doppler_end_ = params.lambda_doppler_end;
    lambda_schedule_iters_ = params.lambda_schedule_iters;
    frame_dt_ = params.frame_dt;
    t_vl_x_ = params.t_vl_x;
    t_vl_y_ = params.t_vl_y;
    t_vl_z_ = params.t_vl_z;
    reject_outliers_ = params.reject_outliers;
    outlier_thresh_ = params.outlier_thresh;
    rejection_min_iters_ = params.rejection_min_iters;
    geometric_min_iters_ = params.geometric_min_iters;
    doppler_min_iters_ = params.doppler_min_iters;
    geometric_k_ = params.geometric_k;
    doppler_k_ = params.doppler_k;
    max_corr_distance_ = params.max_corr_distance;
    min_inliers_ = params.min_inliers;
    last_n_frames_ = params.last_n_frames;
    use_voxel_filter_ = params.use_voxel_filter;
    voxel_size_ = params.voxel_size;
   
    // Apply adaptive normal estimation parameters
    normal_estimation_mode_ = params.normal_estimation_mode;
    static_scene_threshold_ = params.static_scene_threshold;
    RCLCPP_INFO(get_logger(), "Set parameter combination %zu/%zu", param_index + 1, parameter_sets_.size());
    RCLCPP_INFO(get_logger(), "Persistence filter: %s (voxel=%.2f, min_frames=%d)",
               enable_persistence_filter_ ? "ENABLED" : "DISABLED",
               persistence_voxel_size_, min_persistence_frames_);
}

/**
 * @brief Initializes enhanced CSV file for comprehensive trajectory logging
 * 
 * Creates timestamped CSV file with complete parameter metadata for each pose.
 * This enables post-processing analysis and correlation between parameters and performance.
 * 
 * @note File includes all active parameters for full experiment reproducibility
 * @warning File creation failures are logged but non-fatal
 */
void DopplerICPStitcher::initialize_csv_file() {
    // Create dedicated directory for ICP pose outputs
    icp_pose_dir_ = "icp_pose";
    std::filesystem::create_directories(icp_pose_dir_);
   
    // Generate unique filename using UNIX timestamp
    auto now = std::chrono::system_clock::now();
    auto now_time_t = std::chrono::system_clock::to_time_t(now);
    std::stringstream filename;
    filename << "execution_time_" << now_time_t << ".csv";
    excel_filename_ = icp_pose_dir_ + "/" + filename.str();
    // Open file with truncation to ensure clean start
    csv_file_.open(excel_filename_, std::ios::out | std::ios::trunc);
    if (!csv_file_.is_open()) {
        RCLCPP_ERROR(get_logger(), "Failed to open output CSV file: %s", excel_filename_.c_str());
        return;
    }
   
    // Comprehensive header with all metadata columns
    csv_file_ << "timestamp,header_frame_id,"
             << "position_x,position_y,position_z,"
             << "orientation_x,orientation_y,orientation_z,orientation_w,"
             << "timestamp,velocity_threshold,downsample_factor,max_iterations,icp_tolerance,"
             << "lambda_doppler_start,lambda_doppler_end,lambda_schedule_iters,frame_dt,"
             << "t_vl_x,t_vl_y,t_vl_z,reject_outliers,outlier_thresh,rejection_min_iters,"
             << "geometric_min_iters,doppler_min_iters,geometric_k,doppler_k,max_corr_distance,"
             << "min_inliers,last_n_frames,frame_timestamp_seconds,frame_timestamp_nanoseconds,"
             << "use_voxel_filter,voxel_size,parameter_set_index,"
             << "normal_estimation_mode,static_scene_threshold,"
             << "enable_persistence_filter,persistence_voxel_size,min_persistence_frames\n";
    csv_file_.flush();
   
    RCLCPP_INFO(get_logger(), "Initialized enhanced trajectory CSV file: %s", excel_filename_.c_str());
    RCLCPP_INFO(get_logger(), "All parameter combinations will be saved to this file");
}

/**
 * @brief Initializes execution logs for performance monitoring and debugging
 * 
 * Creates separate CSV file for frame-by-frame processing statistics including
 * point counts, iteration usage, and timing information for performance analysis.
 * 
 * @note Logs are essential for algorithm tuning and bottleneck identification
 */
void DopplerICPStitcher::initialize_logs_file() {
    // Create dedicated logs directory
    logs_dir_ = "logs";
    std::filesystem::create_directories(logs_dir_);
   
    // Generate timestamped log filename
    auto now = std::chrono::system_clock::now();
    auto now_time_t = std::chrono::system_clock::to_time_t(now);
    std::stringstream filename;
    filename << "logs_time_execution_" << now_time_t << ".csv";
    logs_filename_ = logs_dir_ + "/" + filename.str();
   
    logs_file_.open(logs_filename_, std::ios::out | std::ios::trunc);
    if (!logs_file_.is_open()) {
        RCLCPP_ERROR(get_logger(), "Failed to open logs CSV file: %s", logs_filename_.c_str());
        return;
    }
   
    // Performance monitoring header
    logs_file_ << "frame_index,filename,initial_points,filtered_points,"
              << "points_after_persistence,iterations_used,processing_time_ms\n";
    logs_file_.flush();
   
    RCLCPP_INFO(get_logger(), "Initialized execution logs file: %s", logs_filename_.c_str());
}

/**
 * @brief Saves frame processing statistics to execution logs CSV file
 * 
 * Records comprehensive performance metrics for each processed frame including:
 * - Point cloud size evolution through pipeline stages
 * - Computational performance and iteration counts
 * - Processing timing for bottleneck analysis
 * 
 * @param stats FrameStats structure containing all performance metrics
 * 
 * @note Flushes after each write to ensure data persistence in case of crashes
 * @warning Method returns silently if logs file is not properly opened
 */
void DopplerICPStitcher::save_frame_stats_to_logs(const FrameStats& stats) {
    if (!logs_file_.is_open()) return;
    // Configure precision for consistent numerical formatting
    logs_file_ << std::fixed << std::setprecision(6);
    // Write comprehensive frame statistics
    logs_file_ << stats.frame_index << ","
              << stats.filename << ","
              << stats.initial_points << ","
              << stats.filtered_points << ","
              << current_frame_points_after_persistence_ << "," 
              << stats.iterations_used << ","
              << stats.processing_time_ms << "\n";
   
    logs_file_.flush();
}
/**
 * @brief Saves pose and comprehensive metadata to enhanced trajectory CSV
 * 
 * Records each estimated pose with complete parameter context for full experiment
 * reproducibility. Essential for post-processing analysis and parameter optimization.
 * 
 * @param frame_idx Sequential frame index for temporal tracking
 * @param pose 4x4 transformation matrix representing estimated pose
 * @param frame_data Original point cloud data for timestamp preservation
 * 
 * @note Includes all active parameters enabling correlation analysis
 * @warning Large datasets may benefit from buffered writes for performance
 */
void DopplerICPStitcher::save_pose_to_csv(size_t frame_idx, const Eigen::Matrix4d& pose, const PointCloudData& frame_data) {
    if (!csv_file_.is_open()) return;
    auto current_time = now();
   
    // Extract position and orientation from transformation matrix
    Eigen::Vector3d position = pose.block<3,1>(0,3);
    Eigen::Quaterniond quat(pose.block<3,3>(0,0));
   
    // Configure numerical precision for consistent data formatting
    csv_file_ << std::fixed << std::setprecision(6);
    csv_file_ << current_time.seconds() << ","
              << "map,"  // Reference frame ID
              << position.x() << ","
              << position.y() << ","
              << position.z() << ","
              << quat.x() << ","
              << quat.y() << ","
              << quat.z() << ","
              << quat.w() << ","
              << std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count() << ","
              << velocity_threshold_ << ","
              << downsample_factor_ << ","
              << max_iterations_ << ","
              << icp_tolerance_ << ","
              << lambda_doppler_start_ << ","
              << lambda_doppler_end_ << ","
              << lambda_schedule_iters_ << ","
              << frame_dt_ << ","
              << t_vl_x_ << ","
              << t_vl_y_ << ","
              << t_vl_z_ << ","
              << (reject_outliers_ ? "true" : "false") << ","
              << outlier_thresh_ << ","
              << rejection_min_iters_ << ","
              << geometric_min_iters_ << ","
              << doppler_min_iters_ << ","
              << geometric_k_ << ","
              << doppler_k_ << ","
              << max_corr_distance_ << ","
              << min_inliers_ << ","
              << last_n_frames_ << ","
              << frame_data.frame_timestamp_seconds << ","
              << frame_data.frame_timestamp_nanoseconds << ","
              // Filtering parameters
              << (use_voxel_filter_ ? "true" : "false") << ","
              << voxel_size_ << ","
              << current_param_index_ << ","
              // Normal estimation parameters
              << current_normal_mode_ << ","
              << static_scene_threshold_ << ","
              << (enable_persistence_filter_ ? "true" : "false") << ","
              // Persistence filter parameters
              << persistence_voxel_size_ << ","
              << min_persistence_frames_ << "\n";
    // Ensure data is written to disk
    csv_file_.flush();
}
/**
 * @brief Analyzes point cloud geometry to classify scene as static or dynamic
 * 
 * Uses z-axis variance as a discriminator between flat static environments
 * and varied dynamic terrain. This classification drives adaptive normal
 * estimation strategies for optimal ICP performance.
 * 
 * @param points Point cloud coordinates matrix (N x 3)
 * @return bool True if scene is classified as static, false for dynamic
 * 
 * @note Static scenes typically have z-variance < threshold (flat environments)
 * @warning Empty point clouds are conservatively classified as static
 * @complexity O(n) where n is number of points
 */
bool DopplerICPStitcher::is_static_scene(const Eigen::MatrixXd& points) {
    if (points.rows() == 0) return true;
   
    // Calculate variance in z-axis
    double z_mean = 0.0;
    for (int i = 0; i < points.rows(); ++i) {
        z_mean += points(i, 2);
    }
    z_mean /= points.rows();
   
    double z_variance = 0.0;
    for (int i = 0; i < points.rows(); ++i) {
        z_variance += (points(i, 2) - z_mean) * (points(i, 2) - z_mean);
    }
    z_variance /= points.rows();
   
    // Classification based on z-variance threshold
    // Static scenes: low z-variance (flat environments like indoor floors)
    // Dynamic scenes: high z-variance (varied terrain during movement)
    bool is_static = z_variance < static_scene_threshold_;
   
    RCLCPP_DEBUG(get_logger(), "Scene detection - Z variance: %.4f, Static: %s",
                 z_variance, is_static ? "true" : "false");
   
    return is_static;
}
/**
 * @brief Initiates asynchronous preprocessing of specified frame
 * 
 * Starts background preprocessing pipeline for next frame while current frame
 * is being processed. This overlapping execution significantly improves
 * overall pipeline throughput and reduces latency.
 * 
 * @param frame_idx Index of frame to preprocess asynchronously
 * 
 * @note Uses std::atomic flag for thread-safe state management
 * @warning Frame index bounds are checked to prevent out-of-range access
 */
void DopplerICPStitcher::start_async_preprocessing(size_t frame_idx) {
    if (frame_idx >= frame_files_.size()) return;
    // Set preprocessing flag atomically
    preprocessing_next_frame_.store(true);
    // Load raw frame and start asynchronous preprocessing
    PointCloudData raw_frame = load_frame(frame_files_[frame_idx]);
    next_frame_future_ = preprocessor_->preprocessAsync(raw_frame);
    RCLCPP_DEBUG(get_logger(), "Started async preprocessing for frame %zu", frame_idx);
}
/**
 * @brief Retrieves results from asynchronous preprocessing pipeline
 * 
 * Blocks until preprocessing is complete and returns the processed frame.
 * Includes timing measurements for performance monitoring and fallback
 * to synchronous loading if no preprocessing was initiated.
 * 
 * @return PointCloudData Preprocessed frame ready for ICP registration
 * 
 * @note Includes wait time measurement for pipeline optimization
 * @warning Blocking call - should be timed to minimize pipeline stalls
 */
PointCloudData DopplerICPStitcher::get_next_preprocessed_frame() {
    // Check if preprocessing was actually initiated
    if (!preprocessing_next_frame_.load()) {
        RCLCPP_WARN(get_logger(), "No frame being preprocessed, loading synchronously");
        return load_frame(frame_files_[frame_idx_]);
    }
    // Measure wait time for performance optimization
    auto start_wait = std::chrono::high_resolution_clock::now();
    // Block until preprocessing completes and get result
    PointCloudData result = next_frame_future_.get();
    preprocessing_next_frame_.store(false);
   
    auto end_wait = std::chrono::high_resolution_clock::now();
    auto wait_time = std::chrono::duration_cast<std::chrono::milliseconds>(end_wait - start_wait);
    RCLCPP_DEBUG(get_logger(), "Retrieved preprocessed frame, wait time: %ld ms", wait_time.count());
   
    return result;
}
/**
 * @brief Comprehensive CSV point cloud loader with robust parsing
 * 
 * Handles multiple CSV formats with flexible column naming and automatic
 * timestamp extraction. Implements velocity-based filtering and efficient
 * memory management for large datasets.
 * 
 * @param filename Path to CSV file containing point cloud data
 * @return PointCloudData Structured point cloud with metadata
 * 
 * @note Supports multiple velocity column names: "v_radial", "radial_vel", "v"
 * @exception std::invalid_argument Handles malformed numerical data gracefully
 */
PointCloudData DopplerICPStitcher::load_frame(const std::string& filename) {
    PointCloudData data;
    std::ifstream in(filename);
    // File access validation
    if (!in.is_open()) {
        RCLCPP_ERROR(get_logger(), "Cannot open file: %s", filename.c_str());
        return data;
    }
    std::string line;
    if (!std::getline(in, line)) {
        RCLCPP_ERROR(get_logger(), "Empty file or bad header: %s", filename.c_str());
        return data;
    }
    // ================= Phase 1: CSV Header Analysis =================
    
    // Parse and clean header line
    std::vector<std::string> headers;
    std::stringstream ss(line);
    std::string h;
    while (std::getline(ss, h, ',')) {
        // Trim whitespace from header elements
        h.erase(0, h.find_first_not_of(" \t\r\n"));
        h.erase(h.find_last_not_of(" \t\r\n") + 1);
        headers.push_back(h);
    }
    // Identify column indices with flexible naming support
    int x_idx = -1, y_idx = -1, z_idx = -1, v_idx = -1;
    int ts_sec_idx = -1, ts_nsec_idx = -1; 
    for (size_t i = 0; i < headers.size(); ++i) {
        if (headers[i] == "x") x_idx = i;
        else if (headers[i] == "y") y_idx = i;
        else if (headers[i] == "z") z_idx = i;
        else if (headers[i] == "v_radial" || headers[i] == "radial_vel" || headers[i] == "v") v_idx = i;
        // Timestamp column detection for temporal analysis
        else if (headers[i] == "frame_timestamp_seconds") ts_sec_idx = i;
        else if (headers[i] == "frame_timestamp_nanoseconds") ts_nsec_idx = i;
    }
    // Validate essential columns exist
    if (x_idx < 0 || y_idx < 0 || z_idx < 0 || v_idx < 0) {
        RCLCPP_ERROR(get_logger(), "Missing required columns in CSV: %s", filename.c_str());
        return data;
    }
    // ================= Phase 2: Efficient Data Loading =================
    
    // Pre-allocate temporary storage for performance
    std::vector<Eigen::Vector4d> temp_data;
    temp_data.reserve(10000);  // Optimized for typical point cloud sizes
   
    size_t total_points = 0;
    size_t filtered_points = 0; 
    bool first_valid_row = true; // Flag for timestamp extraction
    // Process each data row with comprehensive validation
    while (std::getline(in, line)) {
        if (line.empty()) continue;
        std::stringstream sl(line);
        std::vector<std::string> row;
        std::string val;
        // Parse comma-separated values
        while (std::getline(sl, val, ',')) row.push_back(val);
        // Validate row has sufficient columns for required data
        if (row.size() <= static_cast<size_t>(std::max({x_idx, y_idx, z_idx, v_idx}))) continue;
        total_points++;
        double v_radial = 0.0;
        try {
            // Extract and validate radial velocity
            v_radial = std::stod(row[v_idx]);
           
            // Extract timestamps from first valid row only (efficiency)
            if (first_valid_row) {
                if (ts_sec_idx != -1 && ts_sec_idx < static_cast<int>(row.size())) {
                    data.frame_timestamp_seconds = std::stod(row[ts_sec_idx]);
                }
                if (ts_nsec_idx != -1 && ts_nsec_idx < static_cast<int>(row.size())) {
                    data.frame_timestamp_nanoseconds = std::stod(row[ts_nsec_idx]);
                }
                first_valid_row = false;
            }
        } catch (...) { continue; }  // Skip rows with conversion errors
        // Velocity-based filtering using current threshold
        if (std::abs(v_radial) < velocity_threshold_) {
            Eigen::Vector4d entry;
            entry << std::stod(row[x_idx]), std::stod(row[y_idx]), std::stod(row[z_idx]), v_radial;
            temp_data.push_back(entry);
            filtered_points++; 
        }
    }
    // ================= Phase 3: Data Structure Population =================
    size_t N = temp_data.size();
    data.points.resize(N, 3);
    data.velocities.resize(N);
    // Convert temporary storage to efficient Eigen structures
    for (size_t i = 0; i < N; ++i) {
        data.points.row(i) = temp_data[i].head<3>().transpose();
        data.velocities(i) = temp_data[i](3);
    }
    // Store comprehensive statistics for logging and analysis
    current_frame_initial_points_ = total_points;
    current_frame_filtered_points_ = filtered_points;
    RCLCPP_DEBUG(get_logger(), "Loaded %s: %zu points, %zu after filtering, timestamps: %.6fs, %.0fns",
                fs::path(filename).filename().c_str(), total_points, filtered_points,
                data.frame_timestamp_seconds, data.frame_timestamp_nanoseconds);
    return data;
}
/**
 * @brief Comprehensive point cloud preprocessing pipeline
 * 
 * Applies multiple preprocessing stages including downsampling, normal estimation,
 * and velocity preservation. Features adaptive strategies based on scene
 * characteristics and parameter configurations.
 * 
 * @param input Raw point cloud data from loading phase
 * @return PointCloudData Fully processed point cloud ready for ICP
 * 
 * @note Implements both voxel and uniform downsampling strategies
 * @warning Returns empty point cloud if downsampling eliminates all points
 */
PointCloudData DopplerICPStitcher::preprocess_point_cloud(const PointCloudData& input) {
    PointCloudData output;
   
    // Use current parameter values
    bool use_voxel = use_voxel_filter_;
    double voxel_size = voxel_size_;
    // ================= Phase 1: Open3D Data Structure Conversion =================
    auto pcd = std::make_shared<open3d::geometry::PointCloud>();
    pcd->points_.reserve(input.points.rows());
    for (int i = 0; i < input.points.rows(); ++i) {
        pcd->points_.push_back(input.points.row(i).transpose());
    }
    std::shared_ptr<open3d::geometry::PointCloud> pcd_down;
   
    // ================= Phase 2: Multi-Strategy Downsampling =================
    
    // Apply appropriate downsampling based on parameter configuration

    if (use_voxel && voxel_size > 0) {
        pcd_down = pcd->VoxelDownSample(voxel_size);
    } else if (downsample_factor_ > 1) {
        pcd_down = pcd->UniformDownSample(downsample_factor_);
    } else {
        pcd_down = pcd;   // No downsampling - use original data
    }
    if (pcd_down->points_.empty()) return output;
    // ================= Phase 3: Adaptive Normal Estimation =================
    bool use_vertical_normals = false;
    std::string current_normal_mode = normal_estimation_mode_;
   
    // Determine normal estimation strategy based on configuration and scene
    if (normal_estimation_mode_ == "vertical") {
        use_vertical_normals = true;
        current_normal_mode = "vertical";
    } else if (normal_estimation_mode_ == "estimated") {
        use_vertical_normals = false;
        current_normal_mode = "estimated";
    } else { // "auto" mode - detect scene type
        bool is_static = is_static_scene(input.points);
        use_vertical_normals = is_static;
        current_normal_mode = use_vertical_normals ? "auto_vertical" : "auto_estimated";
    }
   
    // Store the current normal mode for logging
    current_normal_mode_ = current_normal_mode;
    if (use_vertical_normals) {
        RCLCPP_INFO(get_logger(), "Using VERTICAL NORMALS for static scene");
        // Don't estimate normals, we'll set them to vertical
        pcd_down->normals_.resize(pcd_down->points_.size(), Eigen::Vector3d(0.0, 0.0, 1.0));
    } else {
        RCLCPP_INFO(get_logger(), "Using ESTIMATED NORMALS for dynamic scene");
        // Estimate normals with optimized parameters (existing behavior)
        pcd_down->EstimateNormals(open3d::geometry::KDTreeSearchParamHybrid(5.0, 20));
    }
    // ================= Phase 4: Output Structure Population =================
    size_t M = pcd_down->points_.size();
    output.points.resize(M, 3);
    output.normals.resize(M, 3);
    output.velocities.resize(M);
    // Copy processed points and normals to output
    for (size_t i = 0; i < M; ++i) {
        output.points.row(i) = pcd_down->points_[i];
        output.normals.row(i) = pcd_down->normals_[i];
    }
    // ================= Phase 5: Velocity Attribute Preservation =================
    
    // Map velocities using nearest neighbor if downsampling occurred
    if (pcd_down->points_.size() < pcd->points_.size()) {
        open3d::geometry::KDTreeFlann kdtree(*pcd);
        for (size_t i = 0; i < M; ++i) {
            std::vector<int> indices(1);
            std::vector<double> dists(1);
            kdtree.SearchKNN(pcd_down->points_[i], 1, indices, dists);
            int idx0 = indices[0];
            output.velocities(i) = (idx0 >= 0 && idx0 < input.velocities.size()) ? input.velocities(idx0) : 0.0;
        }
    } else {
        output.velocities = input.velocities;
    }
    output.frame_timestamp_seconds = input.frame_timestamp_seconds;
    output.frame_timestamp_nanoseconds = input.frame_timestamp_nanoseconds;
    // Direct velocity copy if no downsampling applied
    output.precompute();
    return output;
}
/**
 * @brief Computes Huber weights for robust optimization against outliers
 * 
 * Implements Huber loss weighting function that transitions from quadratic
 * to linear behavior beyond threshold k. This provides robustness to outliers
 * while maintaining differentiability for optimization convergence.
 * 
 * @param residuals Vector of residual values from correspondence matching
 * @param k Threshold parameter controlling transition point
 * @return Eigen::VectorXd Weight vector (1.0 for |r|≤k, k/|r| for |r|>k)
 * 
 * @note Small epsilon (1e-12) prevents division by zero in edge cases
 * @warning Threshold k should be tuned based on expected noise characteristics
 */
inline Eigen::VectorXd DopplerICPStitcher::huber_weights(const Eigen::VectorXd& residuals, double k) {
    const double eps = 1e-12;                     // Numerical stability constant
    Eigen::VectorXd abs_r = residuals.cwiseAbs();
    // Huber weighting: quadratic for small residuals, linear for large residuals
    return (abs_r.array() <= k).select(
        Eigen::VectorXd::Ones(residuals.size()),  // Unity weights for inliers
        k / (abs_r.array() + eps)                 // Decreasing weights for outliers
    );
}

std::pair<Eigen::Matrix4d, double> DopplerICPStitcher::doppler_icp(const PointCloudData& source, const PointCloudData& target) {
    auto start_time = std::chrono::high_resolution_clock::now();
   
    // Use current parameter values
    int max_iter = max_iterations_;
    double tol = icp_tolerance_;
    double lambda_start = lambda_doppler_start_;
    double lambda_end = lambda_doppler_end_;
    int lambda_iters = lambda_schedule_iters_;
    double dt = frame_dt_;
    Eigen::Vector3d t_vl(t_vl_x_, t_vl_y_, t_vl_z_);
    bool reject_outliers = reject_outliers_;
    double outlier_thresh = outlier_thresh_;
    int rejection_min_iters = rejection_min_iters_;
    int geometric_min_iters = geometric_min_iters_;
    int doppler_min_iters = doppler_min_iters_;
    double geometric_k = geometric_k_;
    double doppler_k = doppler_k_;
    double max_corr_distance = max_corr_distance_;
    int min_inliers = min_inliers_;
    // Preprocess
    PointCloudData src = preprocess_point_cloud(source);
    PointCloudData tgt = preprocess_point_cloud(target);
    if (src.points.rows() == 0 || tgt.points.rows() == 0) {
        RCLCPP_WARN(get_logger(), "Insufficient points; returning identity");
        return {Eigen::Matrix4d::Identity(), std::numeric_limits<double>::infinity()};
    }
    // Build KD-tree for target
    auto tgt_pcd = std::make_shared<open3d::geometry::PointCloud>();
    tgt_pcd->points_.reserve(tgt.points.rows());
    for (int i = 0; i < tgt.points.rows(); ++i) {
        tgt_pcd->points_.push_back(tgt.points.row(i).transpose());
    }
    open3d::geometry::KDTreeFlann kdtree(*tgt_pcd);
    Eigen::Matrix4d transformation = Eigen::Matrix4d::Identity();
    double prev_error = std::numeric_limits<double>::infinity();
    // Pre-allocate for performance
    int N = src.points.rows();
    std::vector<int> indices(N);
    std::vector<double> dists(N);
    Eigen::VectorXd r_g(N);
    std::vector<bool> mask(N);
    // Use precomputed unit directions
    const Eigen::MatrixXd& d_unit = src.unit_directions;
    Eigen::MatrixXd r_vecs = src.points.rowwise() + t_vl.transpose();
    int actual_iterations_used = 0; // Track actual iterations used
    for (int it = 0; it < max_iter; ++it) {
        actual_iterations_used = it + 1; // Track iteration count
       
        // Lambda scheduling
        double lam = lambda_end;
        if (lambda_iters > 0) {
            double alpha = std::min(1.0, static_cast<double>(it) / lambda_iters);
            lam = lambda_start + (lambda_end - lambda_start) * alpha;
        }
        // Transform source points
        Eigen::Matrix3d R = transformation.block<3,3>(0,0);
        Eigen::Vector3d t = transformation.block<3,1>(0,3);
        Eigen::MatrixXd src_tf = (R * src.points.transpose()).colwise() + t;
        src_tf.transposeInPlace();
        // Find correspondences
        int inlier_count = 0;
        for (int i = 0; i < N; ++i) {
            std::vector<int> idx(1);
            std::vector<double> dist(1);
            Eigen::Vector3d query(src_tf(i, 0), src_tf(i, 1), src_tf(i, 2));
            kdtree.SearchKNN(query, 1, idx, dist);
           
            dists[i] = std::sqrt(dist[0]);
            indices[i] = idx[0];
           
            // Geometric residual
            Eigen::Vector3d src_pt(src_tf(i, 0), src_tf(i, 1), src_tf(i, 2));
            Eigen::Vector3d tgt_pt(tgt.points(idx[0], 0), tgt.points(idx[0], 1), tgt.points(idx[0], 2));
            Eigen::Vector3d tgt_norm(tgt.normals(idx[0], 0), tgt.normals(idx[0], 1), tgt.normals(idx[0], 2));
            r_g(i) = (src_pt - tgt_pt).dot(tgt_norm);
           
            // Determine inliers
            bool geom_in = (dists[i] < max_corr_distance);
            bool doppler_in = (!reject_outliers || (it + 1) < rejection_min_iters ||
                              std::abs(src.velocities(i)) < outlier_thresh);
            mask[i] = geom_in && doppler_in;
            if (mask[i]) inlier_count++;
        }
        if (inlier_count < min_inliers) {
            RCLCPP_WARN(get_logger(), "Insufficient inliers (%d < %d), breaking ICP", inlier_count, min_inliers);
            break;
        }
        // Build linear system
        Eigen::MatrixXd A(inlier_count * 2, 6);
        Eigen::VectorXd b(inlier_count * 2);
       
        // Compute weights
        Eigen::VectorXd w_g = (it + 1) >= geometric_min_iters ?
                              huber_weights(r_g, geometric_k) :
                              Eigen::VectorXd::Ones(N);
       
        Eigen::VectorXd w_d = (it + 1) >= doppler_min_iters ?
                              huber_weights(src.velocities, doppler_k) :
                              Eigen::VectorXd::Ones(N);
        int row_idx = 0;
       
        // Geometric constraints
        for (int j = 0; j < N; ++j) {
            if (!mask[j]) continue;
            Eigen::Vector3d n(tgt.normals(indices[j], 0), tgt.normals(indices[j], 1), tgt.normals(indices[j], 2));
            Eigen::Vector3d p_tf(src_tf(j, 0), src_tf(j, 1), src_tf(j, 2));
            Eigen::Vector3d Jg_omega = -(n.transpose() * skew(p_tf)) * dt;
            Eigen::Vector3d Jg_v = n * dt;
           
            double wg = std::sqrt((1.0 - lam) * w_g(j));
            A.row(row_idx) << Jg_omega.transpose() * wg, Jg_v.transpose() * wg;
            b(row_idx) = -r_g(j) * wg;
            row_idx++;
        }
        // Doppler constraints
        for (int j = 0; j < N; ++j) {
            if (!mask[j]) continue;
            Eigen::Vector3d r_vec_j(r_vecs(j, 0), r_vecs(j, 1), r_vecs(j, 2));
            Eigen::Vector3d d_unit_j(d_unit(j, 0), d_unit(j, 1), d_unit(j, 2));
            Eigen::Vector3d rx_d = r_vec_j.cross(d_unit_j);
            double wd = std::sqrt(lam * w_d(j));
           
            A.row(row_idx) << rx_d.transpose() * wd, d_unit_j.transpose() * wd;
            b(row_idx) = src.velocities(j) * wd;
            row_idx++;
        }
        if (row_idx < 6) {
            RCLCPP_WARN(get_logger(), "Insufficient constraints");
            break;
        }
        // Resize to actual size
        A.conservativeResize(row_idx, 6);
        b.conservativeResize(row_idx);
        // Solve least squares
        Eigen::VectorXd x = A.colPivHouseholderQr().solve(b);
        Eigen::Vector3d omega = x.head<3>();
        Eigen::Vector3d v = x.tail<3>();
        // Update transformation
        Eigen::Matrix4d delta_T = se3_exp(omega, v, dt);
        transformation = delta_T * transformation;
        // Check convergence
        double total_error = 0.0;
        for (int i = 0; i < N; ++i) {
            if (mask[i]) total_error += dists[i] * dists[i];
        }
        total_error = std::sqrt(total_error / inlier_count);
        if (std::abs(prev_error - total_error) < tol) {
            RCLCPP_DEBUG(get_logger(), "Converged at iteration %d", it);
            break;
        }
        prev_error = total_error;
    }
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    RCLCPP_DEBUG(get_logger(), "ICP took %ld ms", duration.count());
    //  Store iterations used for logging
    current_frame_iterations_ = actual_iterations_used;
    return {transformation, prev_error};
}
/**
 * @brief Main frame processing loop - orchestrates entire pipeline
 * 
 * Implements the complete processing pipeline for each frame:
 * 1. Frame rate control and timing management
 * 2. Parameter set progression and switching
 * 3. Asynchronous frame loading and preprocessing
 * 4. Temporal persistence filtering
 * 5. Doppler-enhanced ICP registration
 * 6. Result publishing and logging
 * 
 * @note This is the core method called by the main processing timer
 * @complexity O(n) per frame where n is point cloud size
 * @exception safe - individual frame failures don't stop pipeline
 */
void DopplerICPStitcher::process_next_frame() {
    auto frame_start = std::chrono::steady_clock::now();
    // ================= Phase 1: Frame Rate Control =================
    
    // Maintain consistent frame processing intervals for temporal consistency
    
    if (first_frame_processed_) {
        auto elapsed = std::chrono::duration_cast<std::chrono::duration<double>>(
            frame_start - last_frame_time_);
       
        if (elapsed.count() < target_frame_time_) {
            auto wait_time = std::chrono::duration<double>(target_frame_time_ - elapsed.count());
            auto wait_start = std::chrono::steady_clock::now();
            // Precision sleep to maintain exact frame intervals
            std::this_thread::sleep_for(wait_time);
           
            auto actual_wait = std::chrono::duration_cast<std::chrono::duration<double>>(
                std::chrono::steady_clock::now() - wait_start);
           
            RCLCPP_DEBUG(get_logger(),
                        "Waited %.3f/%.3f seconds to maintain frame rate",
                        actual_wait.count(), wait_time.count());
           
            frame_start = std::chrono::steady_clock::now();
        }
    }
     // ================= Phase 2: Parameter Set Management =================
    if (frame_idx_ >= frame_files_.size()) {
        RCLCPP_INFO(get_logger(), "All frames processed for parameter set %zu", current_param_index_);
       
        // Progress to next parameter set if available
        if (current_param_index_ + 1 < parameter_sets_.size()) {
            // Move to next parameter set
            current_param_index_++;
            set_current_parameters(current_param_index_);
           
            // Reset processing state for new parameter set
            frame_idx_ = 0;
            next_frame_idx_ = 1;
            previous_frame_set_ = false;
            preprocessing_next_frame_.store(false);
            stitched_frames_.clear();
            trajectory_.clear();
            current_pose_ = Eigen::Matrix4d::Identity();
           
            // Reset timing for new parameter set
            last_frame_time_ = std::chrono::steady_clock::now();
            first_frame_processed_ = false;
           
            RCLCPP_INFO(get_logger(), "Starting processing with parameter set %zu/%zu",
                       current_param_index_ + 1, parameter_sets_.size());
           
            // Start preprocessing first frame of new parameter set
            if (frame_files_.size() > 1) {
                start_async_preprocessing(next_frame_idx_);
            }
           
            return;
        } else {
            RCLCPP_INFO(get_logger(), "All parameter combinations processed");
            timer_->cancel();
        }
        return;
    }
    // ================= Phase 3: Frame Data Acquisition =================
    PointCloudData frame_data;
    if (preprocessing_next_frame_.load() && frame_idx_ == next_frame_idx_ - 1) {
        frame_data = get_next_preprocessed_frame();
        RCLCPP_DEBUG(get_logger(), "Using preprocessed frame %zu", frame_idx_);
    } else {
        frame_data = load_frame(frame_files_[frame_idx_]);
        RCLCPP_DEBUG(get_logger(), "Loading frame %zu synchronously", frame_idx_);
    }
    // ================= Phase 4: Temporal Persistence Filtering =================
    
    // Initialize points count before persistence filter
    current_frame_points_after_persistence_ = frame_data.points.rows();
    // Apply temporal persistence filter for noise reduction
    if (enable_persistence_filter_ && frame_idx_ > 5) {
        frame_data = persistence_filter_.filter_non_persistent_points(frame_data);
       
        // Update points count after persistence filter
        current_frame_points_after_persistence_ = frame_data.points.rows();
       
        RCLCPP_INFO(get_logger(),
                   "Persistence filter: %zu → %zu points (%.1f%% kept)",
                   current_frame_filtered_points_, // from load_frame
                   current_frame_points_after_persistence_,
                   (100.0 * current_frame_points_after_persistence_) / current_frame_filtered_points_);
    }
    // ================= Phase 5: Asynchronous Preprocessing Pipeline =================
    
    // Start preprocessing next frame while current frame processes (pipeline optimization)
    if (frame_idx_ + 1 < frame_files_.size() && !preprocessing_next_frame_.load()) {
        next_frame_idx_ = frame_idx_ + 1;
        start_async_preprocessing(next_frame_idx_);
        RCLCPP_DEBUG(get_logger(), "Started async preprocessing for frame %zu", next_frame_idx_);
    }
    // ================= Phase 6: Frame Validation =================
    if (frame_data.points.rows() == 0) {
        RCLCPP_WARN(get_logger(), "Empty frame %zu, skipping", frame_idx_);
        frame_idx_++;
        return;
    }
    // ================= Phase 7: ICP Registration and Pose Estimation =================
    double dt = frame_dt_;
    Eigen::Vector3d lin_acc = Eigen::Vector3d::Zero();
    Eigen::Vector3d ang_vel = Eigen::Vector3d::Zero();
    if (!previous_frame_set_) {
        // First frame initialization
        current_pose_ = Eigen::Matrix4d::Identity();
        stitched_frames_.push_back({frame_data, current_pose_});
        trajectory_.push_back(current_pose_);
        previous_frame_set_ = true;
       
        // Enhanced pose saving with all parameters
        save_pose_to_csv(frame_idx_, current_pose_, frame_data);
       
        // Save frame statistics to logs
        FrameStats stats;
        stats.frame_index = frame_idx_;
        stats.filename = fs::path(frame_files_[frame_idx_]).filename().string();
        stats.parameter_set_index = current_param_index_;
        stats.initial_points = current_frame_initial_points_;
        stats.filtered_points = current_frame_filtered_points_;
        stats.iterations_used = 0; // No ICP for first frame
        auto frame_end = std::chrono::steady_clock::now();
        auto frame_duration = std::chrono::duration_cast<std::chrono::milliseconds>(frame_end - frame_start);
        stats.processing_time_ms = frame_duration.count();
        save_frame_stats_to_logs(stats);
       
        RCLCPP_INFO(get_logger(), "Processed initial frame %zu with param set %zu", frame_idx_, current_param_index_);
    } else {
        // Subsequent frames: perform Doppler-enhanced ICP registration
        auto [transform_sp2tp, error] = doppler_icp(previous_frame_, frame_data);
        Eigen::Matrix4d delta_T = transform_sp2tp.inverse();
        current_pose_ = current_pose_ * delta_T;
        stitched_frames_.push_back({frame_data, current_pose_});
        trajectory_.push_back(current_pose_);
        // Enhanced pose saving with all parameters
        save_pose_to_csv(frame_idx_, current_pose_, frame_data);
        // Maintain sliding window for memory efficiency
        if (last_n_frames_ > 0 && stitched_frames_.size() > static_cast<size_t>(last_n_frames_)) {
            stitched_frames_.erase(stitched_frames_.begin());
        }
        // Compute linear acceleration and angular velocity
        Eigen::Vector3d delta_t = delta_T.block<3,1>(0,3);
        Eigen::Matrix3d delta_R = delta_T.block<3,3>(0,0);
        lin_acc = delta_t / dt;
        Eigen::AngleAxisd aa(delta_R);
        ang_vel = aa.axis() * aa.angle() / dt;
        auto frame_end = std::chrono::steady_clock::now();
        auto frame_duration = std::chrono::duration_cast<std::chrono::milliseconds>(frame_end - frame_start);
        // Save comprehensive frame statistics
        FrameStats stats;
        stats.frame_index = frame_idx_;
        stats.filename = fs::path(frame_files_[frame_idx_]).filename().string();
        stats.parameter_set_index = current_param_index_;
        stats.initial_points = current_frame_initial_points_;
        stats.filtered_points = current_frame_filtered_points_;
        stats.iterations_used = current_frame_iterations_;
        stats.processing_time_ms = frame_duration.count();
        save_frame_stats_to_logs(stats);
       
        RCLCPP_INFO(get_logger(), "Frame %zu: error=%.4f, time=%ld ms, param_set=%zu, iterations=%d",
                   frame_idx_, error, frame_duration.count(), current_param_index_, current_frame_iterations_);
    }
     // ================= Phase 8: State Update and Publishing =================
    previous_frame_ = frame_data;
    publish_pointcloud();
    publish_current_pose();
    publish_trajectory();
    publish_lin_acc_ang_vel(lin_acc, ang_vel);
    publish_tf();
    // Update timing for next frame
    last_frame_time_ = std::chrono::steady_clock::now();
    first_frame_processed_ = true;
   
    frame_idx_++;
}
/**
 * @brief Publishes stitched point cloud combining multiple frames
 * 
 * Transforms all points in the stitching window to the current sensor frame
 * and publishes as a single PointCloud2 message. Includes MCAP recording
 * for data persistence.
 * 
 * @note Uses sliding window mechanism to control memory usage
 * @warning Point cloud transformation uses current_pose_inv for efficiency
 */
void DopplerICPStitcher::publish_pointcloud() {
    if (stitched_frames_.empty()) return;
    // Compute inverse transformation for efficient point cloud stitching
    Eigen::Matrix4d current_pose_inv = current_pose_.inverse();
    std::vector<Eigen::Vector3d> all_points;
    all_points.reserve(stitched_frames_.size() * 1000); // Reserve space for performance
    // Transform all points in stitching window to current sensor frame
    for (const auto& [frame_data, pose] : stitched_frames_) {
        Eigen::Matrix4d relative_transform = current_pose_inv * pose;
        Eigen::Matrix3d R = relative_transform.block<3,3>(0,0);
        Eigen::Vector3d t = relative_transform.block<3,1>(0,3);
        for (int i = 0; i < frame_data.points.rows(); ++i) {
            Eigen::Vector3d pt = R * frame_data.points.row(i).transpose() + t;
            all_points.push_back(pt);
        }
    }
    // Construct PointCloud2 message
    sensor_msgs::msg::PointCloud2 msg;
    msg.header.stamp = now();
    msg.header.frame_id = "sensor";
    msg.height = 1;
    msg.width = all_points.size();
    msg.is_dense = false;
    msg.is_bigendian = false;
    // Define point field structure (x, y, z)
    sensor_msgs::msg::PointField f;
    f.name = "x"; f.offset = 0; f.datatype = sensor_msgs::msg::PointField::FLOAT32;
    f.count = 1; msg.fields.push_back(f);
    f.name = "y"; f.offset = 4; msg.fields.push_back(f);
    f.name = "z"; f.offset = 8; msg.fields.push_back(f);
    msg.point_step = 12;    // 3 floats × 4 bytes
    msg.row_step = msg.point_step * msg.width;
    msg.data.resize(msg.row_step);
    // Copy point data to message buffer
    uint8_t* ptr = msg.data.data();
    for (const auto& p : all_points) {
        float x = static_cast<float>(p.x());
        float y = static_cast<float>(p.y());
        float z = static_cast<float>(p.z());
        std::memcpy(ptr, &x, 4);
        std::memcpy(ptr + 4, &y, 4);
        std::memcpy(ptr + 8, &z, 4);
        ptr += 12;
    }
    // Publish and record
    pointcloud_pub_->publish(msg);
   
    // Record to MCAP
    record_message(msg, "stitched_cloud");
}
/**
 * @brief Publishes current estimated pose as PoseStamped message
 * 
 * Extracts position and orientation from current transformation matrix
 * and publishes with proper timestamp and frame_id for ROS 2 navigation stacks.
 */
void DopplerICPStitcher::publish_current_pose() {
    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.stamp = now();
    pose_msg.header.frame_id = "map";
    // Extract position and orientation from transformation matrix
    Eigen::Vector3d t = current_pose_.block<3,1>(0,3);
    Eigen::Quaterniond q(current_pose_.block<3,3>(0,0));
    pose_msg.pose.position.x = t.x();
    pose_msg.pose.position.y = t.y();
    pose_msg.pose.position.z = t.z();
    pose_msg.pose.orientation.x = q.x();
    pose_msg.pose.orientation.y = q.y();
    pose_msg.pose.orientation.z = q.z();
    pose_msg.pose.orientation.w = q.w();
    pose_pub_->publish(pose_msg);
    record_message(pose_msg, "icp_pose");
}
/**
 * @brief Publishes complete trajectory as PoseArray message
 * 
 * Compiles all historical poses into a single message for visualization
 * and analysis. Useful for trajectory evaluation and loop closure detection.
 */
void DopplerICPStitcher::publish_trajectory() {
    if (trajectory_.empty()) return;
    geometry_msgs::msg::PoseArray pose_array;
    pose_array.header.stamp = now();
    pose_array.header.frame_id = "map";
    pose_array.poses.reserve(trajectory_.size());
    // Convert all trajectory poses to ROS message format
    for (const auto& pose : trajectory_) {
        geometry_msgs::msg::Pose p;
        Eigen::Vector3d t = pose.block<3,1>(0,3);
        Eigen::Quaterniond q(pose.block<3,3>(0,0));
        p.position.x = t.x();
        p.position.y = t.y();
        p.position.z = t.z();
        p.orientation.x = q.x();
        p.orientation.y = q.y();
        p.orientation.z = q.z();
        p.orientation.w = q.w();
        pose_array.poses.push_back(p);
    }
    trajectory_pub_->publish(pose_array);
    record_message(pose_array, "icp_trajectory");
}
/**
 * @brief Publishes computed linear acceleration and angular velocity
 * 
 * Derived from frame-to-frame transformation differences, these metrics
 * provide motion characteristics useful for inertial navigation integration
 * and dynamic scene analysis.
 * 
 * @param lin_acc Computed linear acceleration vector
 * @param ang_vel Computed angular velocity vector
 */
void DopplerICPStitcher::publish_lin_acc_ang_vel(const Eigen::Vector3d& lin_acc, const Eigen::Vector3d& ang_vel) {
    geometry_msgs::msg::Vector3Stamped lin_msg;
    lin_msg.header.stamp = now();
    lin_msg.header.frame_id = "sensor";
    lin_msg.vector.x = lin_acc.x();
    lin_msg.vector.y = lin_acc.y();
    lin_msg.vector.z = lin_acc.z();
    lin_acc_pub_->publish(lin_msg);
    geometry_msgs::msg::Vector3Stamped ang_msg;
    ang_msg.header.stamp = now();
    ang_msg.header.frame_id = "sensor";
    ang_msg.vector.x = ang_vel.x();
    ang_msg.vector.y = ang_vel.y();
    ang_msg.vector.z = ang_vel.z();
    ang_vel_pub_->publish(ang_msg);
    // Record both messages for later analysis
    record_message(lin_msg, "linear_acceleration");
    record_message(ang_msg, "angular_velocity");
}
/**
 * @brief Publishes transform between map and sensor frames
 * 
 * Essential for ROS 2 TF system, enabling other nodes to transform coordinates
 * between the global map frame and the moving sensor frame.
 */
void DopplerICPStitcher::publish_tf() {
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = now();
    t.header.frame_id = "map";
    t.child_frame_id = "sensor";
   
    t.transform.translation.x = current_pose_(0, 3);
    t.transform.translation.y = current_pose_(1, 3);
    t.transform.translation.z = current_pose_(2, 3);
   
    Eigen::Quaterniond q(current_pose_.block<3,3>(0,0));
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();
   
    tf_broadcaster_->sendTransform(t);
}
