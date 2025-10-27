#pragma once
#include <array>
#include <Eigen/Dense>

using Eigen::Matrix3d;

namespace LogHelpers {

// Converts a 3×3 matrix into a column-major array with time as the first element.
// Format: [t, R00, R10, R20, R01, R11, R21, R02, R12, R22]
inline std::array<double, 10> flattenWithTime(double t, const Matrix3d& R)
{
    std::array<double, 10> row;
    row[0] = t;

    int k = 1;
    for (int j = 0; j < 3; ++j) {         // iterate columns first
        for (int i = 0; i < 3; ++i) {     // then rows
            row[k++] = R(i, j);
        }
    }
    return row;
}

} // namespace LogHelpers
