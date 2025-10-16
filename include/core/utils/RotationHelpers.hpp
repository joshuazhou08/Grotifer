#pragma once
#include <Eigen/Dense>

namespace RotationHelpers {
    // Compute incremental rotation matrix between two body orientations.
    inline Eigen::Matrix3d BodyToBody(const Eigen::Matrix3d& prevRotMat,
                                      const Eigen::Matrix3d& curRotMat)
    {
        return curRotMat * prevRotMat.transpose();
    }
}