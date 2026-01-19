#pragma once

#include "core/utils/RotationHelpers.hpp"
#include <vector>
#include <queue>
#include <cstdlib>
#include <mutex>
#include <iostream>
#include "dotenv/dotenv.h"
#include <Eigen/Dense>

using Eigen::Matrix3d;
using Eigen::Vector3d;

struct AttitudeConfig
{
    static inline void loadEnvFileOnce()
    {
        static std::once_flag onceFlag;
        std::call_once(onceFlag, []()
                       {
            dotenv::init(dotenv::Preserve);
            const char* sample = std::getenv("X_VELOCITY_K_P");
            if (sample != nullptr) {
                std::cout << "[Config] Loaded .env (X_VELOCITY_K_P=" << sample << ")" << std::endl;
            } else {
                std::cout << "[Config] Loaded .env (X_VELOCITY_K_P not set)" << std::endl;
            } });
    }

    static inline double envOrDefault(const char *name, double defaultValue)
    {
        loadEnvFileOnce();
        const char *value = std::getenv(name);
        if (value == nullptr || *value == '\0')
        {
            return defaultValue;
        }
        char *end = nullptr;
        const double parsed = std::strtod(value, &end);
        if (end == value)
        {
            return defaultValue;
        }
        return parsed;
    }

    // Initial Kick Paramters
    static inline constexpr double iniKickDuration = 1000.0e-3;
    static inline const Vector3d iniTorqueVec{0.04, 0.0, 0.0};

    // Detumbling Parameters
    static inline constexpr double detumblingMaxDuration = 10;

    // --- Flags For Configuring Behavior --- //
    static inline constexpr bool initialKick = false; // Perform initial kick to demonstrate detumbling capabilities

    static inline constexpr double deltaTaskTime = 25e-3;

    // Fans
    static inline constexpr double torqueToSpeed = 40000;

    static inline constexpr double fc = 4.0; // Cut off frequency for angular velocity filtering

    // --- Angular velocity loop gain constants --- //
    static inline double xVelocityK_p = envOrDefault("X_VELOCITY_K_P", 0.2);
    static inline double xVelocityK_i = envOrDefault("X_VELOCITY_K_I", 1.5e-3 * xVelocityK_p);
    static inline double xVelocityhLim = envOrDefault("X_VELOCITY_H_LIM", 0.5);
    static inline double xVelocitylLim = envOrDefault("X_VELOCITY_L_LIM", -0.5);
    static inline double xVelocityK_d = envOrDefault("X_VELOCITY_K_D", 0.0);

    static inline double yVelocityK_p = envOrDefault("Y_VELOCITY_K_P", 0.1);
    static inline double yVelocityK_i = envOrDefault("Y_VELOCITY_K_I", 1.5e-3 * yVelocityK_p);
    static inline double yVelocityhLim = envOrDefault("Y_VELOCITY_H_LIM", 1.0);
    static inline double yVelocitylLim = envOrDefault("Y_VELOCITY_L_LIM", -1.0);
    static inline double yVelocityK_d = envOrDefault("Y_VELOCITY_K_D", 0.0);

    static inline double zVelocityK_p = envOrDefault("Z_VELOCITY_K_P", 0.15);
    static inline double zVelocityK_i = envOrDefault("Z_VELOCITY_K_I", 1.5e-3 * zVelocityK_p);
    static inline double zVelocityhLim = envOrDefault("Z_VELOCITY_H_LIM", 0.5);
    static inline double zVelocitylLim = envOrDefault("Z_VELOCITY_L_LIM", -0.5);
    static inline double zVelocityK_d = envOrDefault("Z_VELOCITY_K_D", 0.0);

    // --- Position loop gain constants --- //

    static inline double xPositionK_p = envOrDefault("X_POSITION_K_P", 0.2);
    static inline double xPositionK_i = envOrDefault("X_POSITION_K_I", 1.5e-2 * xPositionK_p);
    static inline double xPositionhLim = envOrDefault("X_POSITION_H_LIM", 0.5);
    static inline double xPositionlLim = envOrDefault("X_POSITION_L_LIM", -0.5);
    static inline double xPositionK_d = envOrDefault("X_POSITION_K_D", 0.0);

    static inline double yPositionK_p = envOrDefault("Y_POSITION_K_P", 0.2);
    static inline double yPositionK_i = envOrDefault("Y_POSITION_K_I", 1.5e-2 * yPositionK_p);
    static inline double yPositionhLim = envOrDefault("Y_POSITION_H_LIM", 0.5);
    static inline double yPositionlLim = envOrDefault("Y_POSITION_L_LIM", -0.5);
    static inline double yPositionK_d = envOrDefault("Y_POSITION_K_D", 0.1);

    static inline double zPositionK_p = envOrDefault("Z_POSITION_K_P", 0.2);
    static inline double zPositionK_i = envOrDefault("Z_POSITION_K_I", 1.5e-2 * zPositionK_p);
    static inline double zPositionhLim = envOrDefault("Z_POSITION_H_LIM", 0.5);
    static inline double zPositionlLim = envOrDefault("Z_POSITION_L_LIM", -0.5);
    static inline double zPositionK_d = envOrDefault("Z_POSITION_K_D", 0.0);

    // --- Arbitrary Rotation Configuration --- //

    // Enable automatic find sun operation after detumbling
    static inline constexpr bool enableFindSun = false;

    // Enable arbitrary rotations (if false, no additional rotations after find sun)
    static inline constexpr bool enableArbitraryRotations = true;

    // Queue of rotation commands to execute sequentially
    static inline constexpr double vel = 0.005;
    static inline constexpr double acc = 1.0e-3;
    static inline std::vector<RotationCommand> getRotationQueue()
    {
        return {
            // RotationCommand(Vector3d{0.0, 0.0, 1.0}, M_PI / 30.0, vel, acc), // IN RADIANS
        };
    }
};

struct TorpConfig
{

    static inline constexpr double deltaTaskTime = 150e-3;
    static inline constexpr double gearRatio = 31.1;

    // Torp Master Control values
    static inline constexpr double oprVel = 1;   // Operational velocity (RPM)
    static inline constexpr double acc = 0.3;    // For torp max acceleration, 1.0 for mounted weights
    static inline constexpr double tAccDec = 60; // acc/dec time (s), 60 for mounted weights
    static inline constexpr double tCruise = 20; // cruise time (s)
    static inline constexpr double tDeployRetract = 65;
    static inline constexpr double stepperSpeed = (200.0 / 60) * (1.034 / 0.04) * 10000; // stepper motor speed (steps/10000s)

    // R & L Torp PI Control gains
    static inline constexpr double l_kp = 20;
    static inline constexpr double l_ki = 18.5;
    static inline constexpr double l_kd = 0.0;
    static inline constexpr double l_hLim = 32.0;
    static inline constexpr double l_lLim = -32.0;

    static inline constexpr double r_kp = 20;
    static inline constexpr double r_ki = 18.5;
    static inline constexpr double r_kd = 0.0;
    static inline constexpr double r_hLim = 32.0;
    static inline constexpr double r_lLim = -32.0;

    // Run configuration flags
    static inline constexpr bool holdPosAfterDeployFlag = true;
    static inline constexpr bool sensorsOnlyFlag = false;
    static inline constexpr bool controlBodyOnlyFlag = false; // If flag = true, torp state machine will not run
};
