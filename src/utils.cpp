/**
 * @file utils.cpp
 * @brief Utility functions for point cloud processing pipeline
 * 
 * Contains general-purpose utility functions used throughout the Doppler ICP stitcher system.
 * Implements string processing and sorting algorithms optimized for point cloud data management.
 */
#include "doppler_icp_stitcher_open3d_pro2/utils.h"
/**
 * @brief Performs natural sorting of strings containing numerical components
 * 
 * Implements a natural sort algorithm that properly handles numerical substrings within filenames.
 * This is essential for correctly ordering point cloud frames that follow pattern: frame_1.pcd, frame_2.pcd, ..., frame_10.pcd
 * 
 * @param a First string to compare
 * @param b Second string to compare
 * @return bool True if a should come before b in natural sort order
 * 
 * @note Uses regex to extract numerical components for proper numerical comparison
 * 
 * @warning Regular expression matching may have performance implications for large datasets
 *          Consider pre-processing if used in performance-critical loops
 */
// Remove the inline function definitions since they're already in the header
// Only keep the non-inline function

bool natural_sort(const std::string& a, const std::string& b) {
    // Lambda function to extract the first numerical sequence from a string
    auto get_number = [](const std::string& s) -> int {
        std::smatch match;
        // Regex pattern to find first contiguous sequence of digits
        if (std::regex_search(s, match, std::regex("(\\d+)"))) {
            return std::stoi(match[1].str()); // Convert captured digits to integer

        }
        return 0; // Default return if no numbers found
    };
    // Compare based on extracted numerical values
    return get_number(a) < get_number(b);
}
