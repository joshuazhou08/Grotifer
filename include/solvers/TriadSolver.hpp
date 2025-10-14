#pragma once
#include <Eigen/Dense>


namespace TriadSolver {
    // Public API
    Eigen::Matrix3d solve(double thxIncl, double thzIncl,
                          double thySun, double thzSun);
}