#include "solvers/TriadSolver.hpp"
#include <cmath>

using namespace Eigen;

namespace {
    using namespace std;  

    Vector3d FindU_b(double thySun, double thzSun)
    {
        double ux_b = 1.0 / sqrt(1 + pow(tan(thySun), 2) + pow(tan(thzSun), 2));
        double uy_b = ux_b * tan(thySun);
        double uz_b = ux_b * tan(-thzSun);
        return Vector3d(ux_b, uy_b, uz_b);
    }

    Vector3d FindV_b(double thxIncl, double thzIncl)
    {
        double vy_b = 1.0 / sqrt(1 + pow(tan(thxIncl), 2) + pow(tan(thzIncl), 2));
        double vx_b = vy_b * tan(thxIncl);
        double vz_b = vy_b * tan(thzIncl);
        return Vector3d(vx_b, vy_b, vz_b);
    }

    Matrix3d FindM_r()
    {
        Vector3d u_r(1, 0, 0);   // Sun direction
        Vector3d v_r(0, 1, 0);   // Opposite of gravity

        Vector3d q_r = u_r;
        Vector3d r_r = u_r.cross(v_r).normalized();
        Vector3d s_r = q_r.cross(r_r);

        Matrix3d referenceAxes;
        referenceAxes << q_r, r_r, s_r;
        return referenceAxes;
    }

    Matrix3d FindM_b(const Vector3d& u_b, const Vector3d& v_b)
    {
        Vector3d q_b = u_b;
        Vector3d r_b = u_b.cross(v_b).normalized();
        Vector3d s_b = q_b.cross(r_b);

        Matrix3d bodyAxes;
        bodyAxes << q_b, r_b, s_b;
        return bodyAxes;
    }
} 


// ======== public API ========

Matrix3d solve(double thxIncl, double thzIncl,
               double thySun, double thzSun)
{
    Vector3d u_b = FindU_b(thySun, thzSun);
    Vector3d v_b = FindV_b(thxIncl, thzIncl);
    Matrix3d M_r = FindM_r();
    Matrix3d M_b = FindM_b(u_b, v_b);

    Matrix3d attitudeMatrix = M_b * M_r.transpose();
    
    return attitudeMatrix.transpose();  //Not very important, but we should try to figure out why this is returning the transpose of the correct matrix
}