#include "solvers/AngularVelocitySolver.hpp"
#include <Eigen/Geometry>
#include <cmath>

using namespace Eigen;

namespace AngularVelocitySolver {

namespace {
    using namespace std;

    Matrix3d RotMatBodyToBody(const Matrix3d& prevRotMat, const Matrix3d& curRotMat)
    {
        return curRotMat * prevRotMat.transpose();
    }

} // anonymous namespace

// ======== public API ========

Vector3d solve(const Matrix3d& prevRotMat,
               const Matrix3d& curRotMat,
               double deltaT)
{
    Matrix3d rotMat = RotMatBodyToBody(prevRotMat, curRotMat);

    AngleAxisd angleAxis(rotMat);
    Vector3d rotAxis = angleAxis.axis();
    rotAxis(0) = -rotAxis(0); // corrects sign on x-axis (nobody knows why)
    double rotAngle = angleAxis.angle();

    return rotAxis * rotAngle / deltaT;
}

} 