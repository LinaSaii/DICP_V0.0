/**
 * @file async_preprocessor.h
 * @brief Asynchronous point cloud preprocessing pipeline
 * 
 * Implements producer-consumer pattern for parallel point cloud preprocessing
 * to maximize pipeline throughput and reduce processing latency.
 */
#ifndef ASYNC_PREPROCESSOR_H
#define ASYNC_PREPROCESSOR_H

#include "point_cloud_data.h"
#include <thread>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <functional>
#include <future>
#include <atomic>

// ================= Asynchronous Preprocessor Class =================

/**
 * @brief Manages asynchronous preprocessing of point cloud data
 * 
 * Uses a dedicated worker thread to process point cloud preprocessing tasks
 * in parallel with the main ICP pipeline, enabling overlapping execution
 * and improved overall performance.
 */
class AsyncPreprocessor {
public:
    /**
     * @brief Constructs preprocessor and starts worker thread
     */
    AsyncPreprocessor();
    /**
     * @brief Destructor - ensures graceful thread termination
     */
    ~AsyncPreprocessor();
    /**
     * @brief Submits point cloud for asynchronous preprocessing
     * 
     * Enqueues preprocessing task and returns immediately with a future object.
     * The actual preprocessing occurs in the background worker thread.
     * 
     * @param data Point cloud data to preprocess asynchronously
     * @return std::future<PointCloudData> Future for retrieving processed results
     * @throws std::runtime_error if preprocessor is in stopped state
     */
    std::future<PointCloudData> preprocessAsync(const PointCloudData& data);

private:
    /**
     * @brief Worker thread main processing loop
     * 
     * Continuously processes tasks from the queue in FIFO order.
     * Implements efficient waiting using condition variables.
     */
    void processLoop();
   
    std::thread worker_;
    std::queue<std::function<void()>> tasks_;
    std::mutex queue_mutex_;
    std::condition_variable condition_;
    bool stop_;
};

#endif // ASYNC_PREPROCESSOR_H
