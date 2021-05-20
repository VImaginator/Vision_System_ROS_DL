#include <tf2/LinearMath/Quaternion.h>
#include "obstacledetect.h"


// Normal threshold, |z_norm_| > th_z_norm_ means the point is from plane
float th_z_norm_ = 0.7;

// Region growing threshold
float th_smooth_ = 8;

// Voxel grid threshold
float th_leaf_ = 0.015;
// th_deltaz_ must 2 times bigger than th_leaf_
float th_deltaz_ = 2 * th_leaf_;
float th_ratio_ = 5 * th_leaf_; // flatness ratio max value of plane

// Depth threshold
float th_max_depth_ = 1.3;

ObstacleDetect::ObstacleDetect(bo