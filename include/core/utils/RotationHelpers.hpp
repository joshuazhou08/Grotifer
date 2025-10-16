#pragma once
#include <Eigen/Dense>
#include <cmath>

namespace RotationHelpers {
    // Compute incremental rotation matrix between two body orientations.
    inline Eigen::Matrix3d BodyToBody(const Eigen::Matrix3d& prevRotMat,
                                      const Eigen::Matrix3d& curRotMat)
    {
        return curRotMat * prevRotMat.transpose();
    }

    // Convert angle from degrees to radians
    inline double Deg2Rad(double angleDeg) {
        return angleDeg * M_PI / 180.0;
    }

    // Convert angle from radians to degrees
    inline double Rad2Deg(double angleRad) {
        return angleRad * 180.0 / M_PI;
    }
}