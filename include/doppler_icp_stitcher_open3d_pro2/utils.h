/**
 * @file utils.h
 * @brief Mathematical utilities and helper functions for 3D transformations
 * 
 * Provides essential mathematical operations for 3D geometry including:
 * - Skew-symmetric matrix operations for cross products
 * - Exponential maps for SO(3) and SE(3) Lie groups
 * - String processing for natural sorting algorithms
 */
#ifndef UTILS_H
#define UTILS_H

#include <Eigen/Dense>
#include <string>
#include <regex>
// ================= Mathematical Helper Functions =================

/**
 * @brief Computes the skew-symmetric matrix of a 3D vector
 * 
 * Converts a 3D vector to its skew-symmetric matrix representation for cross product operations.
 * This is essential for Lie algebra operations in 3D transformations.
 * 
 * @param v Input 3D vector [vx, vy, vz]
 * @return Eigen::Matrix3d Skew-symmetric matrix where [v]× w = v × w
 * 
 * @note Skew-symmetric matrices satisfy A = -Aᵀ (antisymmetric property)
 * @see Used in exponential maps for rotation and transformation computations
 */

inline Eigen::Matrix3d skew(const Eigen::Vector3d& v) {
    Eigen::Matrix3d S;
    S << 0.0, -v(2), v(1),
         v(2), 0.0, -v(0),
        -v(1), v(0), 0.0;
    return S;
}
/**
 * @brief Computes the exponential map for SO(3) Lie algebra
 * 
 * Converts an angular velocity vector (Lie algebra element) to a rotation matrix
 * (Lie group element) using the Rodrigues' rotation formula.
 * 
 * @param omega Angular velocity vector [ωx, ωy, ωz] in radians per second
 * @return Eigen::Matrix3d 3x3 rotation matrix in SO(3)
 * 
 * @note Implements Rodrigues' formula: R = I + sin(θ)[k]× + (1-cos(θ))[k]×²
 * @warning Numerical stability check for small angles (θ < 1e-12)
 * @complexity O(1) - constant time operation
 */
inline Eigen::Matrix3d exp_so3(const Eigen::Vector3d& omega) {
    double theta = omega.norm();
    // Identity for zero rotation (numerical stability)
    if (theta < 1e-12) return Eigen::Matrix3d::Identity();
    // Normalize axis and compute skew-symmetric matrix
    Eigen::Vector3d k = omega / theta;
    Eigen::Matrix3d K = skew(k);
    // Rodrigues' rotation formula
    return Eigen::Matrix3d::Identity() + std::sin(theta) * K +
           (1.0 - std::cos(theta)) * (K*K);
}
/**
 * @brief Computes the exponential map for SE(3) Lie algebra
 * 
 * Converts a twist vector (angular and linear velocities) to a homogeneous
 * transformation matrix using the exponential map for rigid body motions.
 * 
 * @param omega Angular velocity vector [ωx, ωy, ωz]
 * @param v Linear velocity vector [vx, vy, vz]
 * @param dt Time delta for integration
 * @return Eigen::Matrix4d 4x4 homogeneous transformation matrix in SE(3)
 * 
 * @note Combines rotational (SO(3)) and translational components
 * @see Used in ICP for incremental pose updates
 */
inline Eigen::Matrix4d se3_exp(const Eigen::Vector3d& omega, const Eigen::Vector3d& v, double dt) {
    // Compute rotation component
    Eigen::Matrix3d R = exp_so3(omega*dt);
    // Compute translation component (simplified for small dt)
    Eigen::Vector3d t = v*dt;
    // Construct homogeneous transformation matrix
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T.block<3,3>(0,0) = R;
    T.block<3,1>(0,3) = t;
    return T;
}

/**
 * @brief Performs natural sorting of strings containing numerical components
 * 
 * Compares strings using natural order (human-readable) rather than lexicographical order.
 * Essential for correctly ordering sequential files: frame1, frame2, ..., frame10
 * 
 * @param a First string to compare
 * @param b Second string to compare
 * @return bool True if a comes before b in natural sort order
 * 
 * @note Implementation provided in utils.cpp
 * @example "frame2" < "frame10" (numerically 2 < 10, not lexicographically)
 */
bool natural_sort(const std::string& a, const std::string& b);

#endif // UTILS_H
