#pragma once
#include <Eigen/Dense>
#include <cmath>

using namespace Eigen;

// Structure to define a rotation command
struct RotationCommand
{
    Vector3d axis;       // Rotation axis (will be normalized)
    double angle;        // Rotation angle in radians
    double velocity;     // Maximum velocity for this move [rad/s]
    double acceleration; // Acceleration for this move [rad/s^2]

    RotationCommand(Vector3d rotAxis, double rotAngle, double vel = 0.01, double accel = 2.0e-3)
        : axis(rotAxis), angle(rotAngle), velocity(vel), acceleration(accel) {}

    RotationCommand()   // Default Initializer
        : axis(Eigen::Vector3d::UnitX()), angle(0.0), velocity(0.01), acceleration(2.0e-3) {}

};

namespace RotationHelpers {
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