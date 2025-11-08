/**
 * @file async_preprocessor.cpp
 * @brief Asynchronous point cloud preprocessing pipeline
 * 
 * Implements a producer-consumer pattern for parallel point cloud preprocessing.
 * Enables non-blocking preprocessing operations to maximize pipeline throughput.
 */
#include "doppler_icp_stitcher_open3d_pro2/async_preprocessor.h"
#include "doppler_icp_stitcher_open3d_pro2/point_cloud_data.h"
#include <thread>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <functional>
#include <future>
#include <atomic>

// ================= AsyncPreprocessor Implementation =================
/**
 * @brief Constructs the asynchronous preprocessor and starts worker thread
 * 
 * Initializes the processing pipeline and launches the dedicated worker thread
 * that will handle all preprocessing tasks asynchronously.
 */
AsyncPreprocessor::AsyncPreprocessor() : stop_(false) {
    worker_ = std::thread(&AsyncPreprocessor::processLoop, this);
}
/**
 * @brief Destructor - ensures graceful shutdown of preprocessing pipeline
 * 
 * Signals the worker thread to stop, waits for completion of pending tasks,
 * and ensures proper thread termination to prevent resource leaks.
 */
AsyncPreprocessor::~AsyncPreprocessor() {
    {   // Acquire lock and set stop flag atomically
        std::unique_lock<std::mutex> lock(queue_mutex_);
        stop_ = true;
        // Notify worker thread to wake up and check termination condition

    }
    condition_.notify_all();
    // Wait for worker thread to complete current task and terminate
    if (worker_.joinable()) {
        worker_.join();
    }
}
/**
 * @brief Submits a point cloud for asynchronous preprocessing
 * 
 * Enqueues a preprocessing task and returns immediately with a future object.
 * The actual preprocessing occurs in the background worker thread.
 * 
 * @param data Reference to the point cloud data to be preprocessed
 * @return std::future<PointCloudData> Future object for retrieving processed results
 * @throws std::runtime_error if preprocessor is in stopped state
 * 
 * @note This method is thread-safe and can be called from multiple threads
 */
std::future<PointCloudData> AsyncPreprocessor::preprocessAsync(const PointCloudData& data) {
    // Create packaged task to wrap the preprocessing operation
    auto task = std::make_shared<std::packaged_task<PointCloudData()>>(
        [data]() -> PointCloudData {
            // Current implementation: copy input and trigger precomputation
            // This serves as a template for more complex preprocessing pipelines
            PointCloudData result = data;
            result.precompute(); // Trigger any necessary precomputation steps
            return result;
        }
    );
    // Extract future before enqueueing to avoid race conditions
    std::future<PointCloudData> result = task->get_future();
    {   // Critical section: enqueue task thread-safely
        std::unique_lock<std::mutex> lock(queue_mutex_);
        if (stop_) {
            throw std::runtime_error("enqueue on stopped AsyncPreprocessor");
        }
        tasks_.emplace([task]() { (*task)(); }); // Wrap in void() lambda

    }
    // Notify worker thread that new task is available
    condition_.notify_one();
    return result;
}
/**
 * @brief Worker thread main processing loop
 * 
 * Continuously processes tasks from the queue in FIFO order.
 * Implements efficient waiting using condition variables to avoid busy-waiting.
 * Gracefully handles shutdown signals and processes all pending tasks before termination.
 */
void AsyncPreprocessor::processLoop() {
    while (true) {
        // Wait for tasks or shutdown signal
        std::function<void()> task;
        {
            std::unique_lock<std::mutex> lock(queue_mutex_);
            condition_.wait(lock, [this] {
                return stop_ || !tasks_.empty();   // Wake up if stop signaled or tasks available
            });
            // Termination condition: stop signaled and no pending tasks
            if (stop_ && tasks_.empty()) {
                return;
            }
            // Extract next task from queue
            task = std::move(tasks_.front());
            tasks_.pop();
        }
        // Execute task outside of critical section to maximize concurrency
        task();
    }
}
