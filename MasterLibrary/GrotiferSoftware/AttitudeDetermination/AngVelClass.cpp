
#include "AttitudeDetermination.hpp"

using Eigen::AngleAxisd;
using Eigen::Matrix3d;
using Eigen::Vector3d;

using namespace std;

#define PI 3.14159265358979323846

// depecrated, builtin eigen
Vector3d AngularVel::RotAxisFromRotMat(Matrix3d rotMat)
{
    Vector3d u;
    u << rotMat(2, 1) - rotMat(1, 2), rotMat(0, 2) - rotMat(2, 0), rotMat(1, 0) - rotMat(0, 1);
    u.normalize();
    return u;
}

// depecrated, builtin eigen
double AngularVel::RotAngleAboutRotAxis(Matrix3d rotMat)
{

    // manually calculate traace as we had this issue in MATLAB

    double trace{rotMat.trace()};

    double rotAngle{acos((trace - 1) / 2)};
    return rotAngle; // radians
}

Matrix3d AngularVel::RotMatBodyToBody(Matrix3d prevRotMat, Matrix3d curRotMat)
{
    Matrix3d incRotMat;
    incRotMat = curRotMat * prevRotMat.transpose();

    return incRotMat;
}

Vector3d AngularVel::GetAngularVelVec(Matrix3d prevRotMat, Matrix3d curRotMat, double deltaT)
{
    Matrix3d rotMat{RotMatBodyToBody(prevRotMat, curRotMat)};

    AngleAxisd angleAxis(rotMat);
    Vector3d rotAxis{angleAxis.axis()};
    rotAxis(0) = -1 * rotAxis(0); // we are for some reason getting the negative direction on x
    double rotAngle{angleAxis.angle()};

    Vector3d angVel;
    angVel = rotAxis * rotAngle / deltaT;

    return angVel;
}
