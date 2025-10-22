#pragma once
#include <Eigen/Dense>
#include <cmath>

namespace RotationHelpers {
    using namespace Eigen;
    // Compute incremental rotation matrix between two body orientations.
    inline Matrix3d BodyToBody(const Matrix3d& prevRotMat,
                                      const Matrix3d& curRotMat)
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

    // Helper function to go from rotation matrix to rotation vector
    inline Vector3d calculateRotationVector(const Matrix3d &rotMat)
    {
        AngleAxisd angleAxis{rotMat};

        Vector3d rotAxis = angleAxis.axis();
        rotAxis(0) = -rotAxis(0); // needs to be flipped because of eigen bug
        double rotAngle = angleAxis.angle();

        return rotAxis * rotAngle;
    }

    inline Vector3d calculateRotationVector(const Matrix3d &fromMatrix, const Matrix3d &toMatrix)
    {
        Matrix3d rotMat = BodyToBody(fromMatrix, toMatrix);
        return calculateRotationVector(rotMat);
    }

    // Helper function to go from rotation vector to rotation matrix
    inline Matrix3d calculateRotationMatrix(const Vector3d &rotVec)
    {
        Vector3d rotAxis = rotVec.normalized();
        rotAxis(0) = -rotAxis(0); // needs to be flipped because of eigen bug
        double rotAngle = rotVec.norm();

        AngleAxisd angleAxis{rotAngle, rotAxis};
        return angleAxis.toRotationMatrix();
    }

}