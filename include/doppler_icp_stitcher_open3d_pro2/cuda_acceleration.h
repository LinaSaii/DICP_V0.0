/**
 * @file cuda_acceleration.h
 * @brief CUDA-accelerated computational kernels interface
 * 
 * Provides GPU-accelerated implementations for computationally intensive
 * operations in the point cloud processing pipeline. Includes both C++ class
 * interface and external C function declarations for flexibility.
 */
#pragma once
#include <memory>
#include <future>

// Forward declaration to avoid including Eigen in header
struct PointCloudData;
/**
 * @brief GPU memory representation of point cloud data
 * 
 * Stores point cloud data in GPU memory for efficient CUDA kernel execution.
 * Manages device memory allocation and deallocation.
 */
struct CudaPointCloud {
    float* points;
    float* normals;
    float* velocities;
    int num_points;
    CudaPointCloud() : points(nullptr), normals(nullptr), velocities(nullptr), num_points(0) {}
};
/**
 * @brief Singleton class for CUDA-accelerated computations
 * 
 * Manages GPU resources and provides asynchronous interfaces for
 * computationally intensive operations in the point cloud pipeline.
 * Implements thread pool for overlapping CPU-GPU execution.
 */
class CudaAccelerator {
public:
    /**
     * @brief Gets the singleton instance of CudaAccelerator
     * @return CudaAccelerator& Reference to singleton instance
     */
    static CudaAccelerator& getInstance();
   
    // ================= Asynchronous Processing Interface =================
    
    /**
     * @brief Submits point cloud for asynchronous GPU preprocessing
     * @param input CPU point cloud data to process
     * @return std::future<PointCloudData> Future for retrieving processed results
     */
    std::future<PointCloudData> asyncPreprocess(const PointCloudData& input);
   
     // ================= CUDA-Accelerated Computational Kernels =================
    
    /**
     * @brief GPU-accelerated nearest neighbor search
     * @param query_points Input query points array
     * @param num_queries Number of query points
     * @param target_points Input target points array
     * @param num_targets Number of target points
     * @param indices Output indices of nearest neighbors
     * @param distances Output distances to nearest neighbors
     */
    void cudaKDTreeSearch(const float* query_points, int num_queries,
                         const float* target_points, int num_targets,
                         int* indices, float* distances);
    /**
     * @brief GPU-accelerated matrix multiplication
     * @param A First input matrix (m x n)
     * @param B Second input matrix (n x k)
     * @param C Output matrix (m x k)
     * @param m Rows in matrix A
     * @param n Columns in A / Rows in B
     * @param k Columns in matrix B
     */
    void cudaMatrixMultiply(const float* A, const float* B, float* C, int m, int n, int k);
    /**
     * @brief GPU-accelerated linear system solver
     * @param A Coefficient matrix (n x n)
     * @param b Right-hand side vector (n)
     * @param x Solution vector (n)
     * @param n System dimension
     */
    void cudaSolveLinearSystem(const float* A, const float* b, float* x, int n);

    // ================= Memory Management Interface =================
    
    /**
     * @brief Uploads point cloud data from CPU to GPU memory
     * @param cpu_data Source CPU point cloud data
     * @param gpu_data Destination GPU point cloud structure
     */
    void uploadPointCloud(const PointCloudData& cpu_data, CudaPointCloud& gpu_data);
    /**
     * @brief Downloads point cloud data from GPU to CPU memory
     * @param gpu_data Source GPU point cloud data
     * @param cpu_data Destination CPU point cloud structure
     */
    void downloadPointCloud(const CudaPointCloud& gpu_data, PointCloudData& cpu_data);
    /**
     * @brief Releases GPU memory allocated for point cloud
     * @param gpu_data GPU point cloud structure to deallocate
     */
    void freePointCloud(CudaPointCloud& gpu_data);
   
private:
    CudaAccelerator();
    ~CudaAccelerator();
    class ThreadPool;
    std::unique_ptr<ThreadPool> thread_pool_;
};

// ================= External C Interface =================

/**
 * @brief External C function for CUDA KD-tree search
 * 
 * Provides C-compatible interface for CUDA-accelerated nearest neighbor search.
 * Can be called from C code or other languages with C bindings.
 * 
 * @param query_points Input query points (x,y,z interleaved)
 * @param num_queries Number of query points
 * @param target_points Input target points (x,y,z interleaved)
 * @param num_targets Number of target points
 * @param indices Output indices of nearest neighbors
 * @param distances Output Euclidean distances
 */
extern "C" void cudaKDTreeSearch(const float* query_points, int num_queries,
                                const float* target_points, int num_targets,
                                int* indices, float* distances);
