#pragma once

#include "core/utils/RotationHelpers.hpp"
#include <vector>
#include <queue>
#include <Eigen/Dense>

using Eigen::Matrix3d;
using Eigen::Vector3d;


struct AttitudeConfig
{

    // Initial Kick Paramters
    static inline constexpr double iniKickDuration = 1000.0e-3;
    static inline const Vector3d iniTorqueVec{0.04, 0.0, 0.0};

    // Detumbling Parameters
    static inline constexpr double detumblingMaxDuration = 10;

    // --- Flags For Configuring Behavior --- //
    static inline constexpr bool initialKick = false; // Perform initial kick to demonstrate detumbling capabilities

    static inline constexpr double deltaTaskTime = 50e-3;

    // Fans
    static inline constexpr double torqueToSpeed = 40000;

    static inline constexpr double fc = 4.0; // Cut off frequency for angular velocity filtering

    // --- Angular velocity loop gain constants --- //
    static inline constexpr double xVelocityK_p = 0.15;
    static inline constexpr double xVelocityK_i = 1.5e-3 * xVelocityK_p;
    static inline constexpr double xVelocityhLim = 0.5;
    static inline constexpr double xVelocitylLim = -0.5;
    static inline constexpr double xVelocityK_d = 0;

    static inline constexpr double yVelocityK_p = 0.1;
    static inline constexpr double yVelocityK_i = 1.5e-3 * yVelocityK_p;
    static inline constexpr double yVelocityhLim = 1.0;
    static inline constexpr double yVelocitylLim = -1.0;
    static inline constexpr double yVelocityK_d = 0;

    static inline constexpr double zVelocityK_p = 0.08;
    static inline constexpr double zVelocityK_i = 1.5e-3 * zVelocityK_p;
    static inline constexpr double zVelocityhLim = 0.5;
    static inline constexpr double zVelocitylLim = -0.5;
    static inline constexpr double zVelocityK_d = 0;

    // --- Position loop gain constants --- //

    static inline constexpr double xPositionK_p = 0.2;
    static inline constexpr double xPositionK_i = 0.2;
    static inline constexpr double xPositionhLim = 0.5;
    static inline constexpr double xPositionlLim = -0.5;
    static inline constexpr double xPositionK_d = 0;

    static inline constexpr double yPositionK_p = 0.2;
    static inline constexpr double yPositionK_i = 1.5e-2 * yPositionK_p;
    static inline constexpr double yPositionhLim = 1.0;
    static inline constexpr double yPositionlLim = -1.0;
    static inline constexpr double yPositionK_d = 0.1;

    static inline constexpr double zPositionK_p = 0.02;
    static inline constexpr double zPositionK_i = 1.5e-3 * zPositionK_p;
    static inline constexpr double zPositionhLim = 0.5;
    static inline constexpr double zPositionlLim = -0.5;
    static inline constexpr double zPositionK_d = 0.1;

    // --- Arbitrary Rotation Configuration --- //

    // Enable automatic find sun operation after detumbling
    static inline constexpr bool enableFindSun = false;

    // Enable arbitrary rotations (if false, no additional rotations after find sun)
    static inline constexpr bool enableArbitraryRotations = true;

    // Queue of rotation commands to execute sequentially
    static inline std::vector<RotationCommand> getRotationQueue()
    {
        return {
            RotationCommand(Vector3d{1.0, 0.0, 0.0}, M_PI / 15.0)// IN RADIANS
            // RotationCommand(Vector3d{1.0, 1.0, 1.0}, M_PI / 12.0, vel, accel), // IN RADIANS
            // RotationCommand(Vector3d{1.0, 0.0, 0.0}, M_PI / 12.0, vel, accel)   // IN RADIANS
        };
    }
};
