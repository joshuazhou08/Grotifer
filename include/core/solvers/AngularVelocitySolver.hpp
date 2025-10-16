#pragma once
#include <Eigen/Dense>

namespace AngularVelocitySolver {
    // Returns the angular velocity vector (rad/s) between two body rotation matrices
    Eigen::Vector3d solve(const Eigen::Matrix3d& prevRotMat,
                          const Eigen::Matrix3d& curRotMat,
                          double deltaT);
}
