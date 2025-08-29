#pragma once

#include "GrotiferMaster.hpp"
#include "Logger.hpp"
#include <vector>
#include <queue>

// Structure to define a rotation command
struct RotationCommand
{
    Vector3d axis;       // Rotation axis (will be normalized)
    double angle;        // Rotation angle in radians
    double velocity;     // Maximum velocity for this move [rad/s]
    double acceleration; // Acceleration for this move [rad/s^2]

    RotationCommand(Vector3d rotAxis, double rotAngle, double vel = 0.015, double accel = 2.0e-3)
        : axis(rotAxis), angle(rotAngle), velocity(vel), acceleration(accel) {}
};
struct AttitudeConfig
{

    // Initial Kick Paramters
    static inline constexpr double iniKickDuration = 250.0e-3;
    static inline const Vector3d iniTorqueVec{0.0, 0.0, 0.1};

    // Detumbling Parameters
    static inline constexpr double detumblingMaxDuration = 10;

    // --- Flags For Configuring Behavior --- //
    static inline constexpr bool initialKick = true; // Perform initial kick to demonstrate detumbling capabilities

    static inline constexpr double deltaTaskTime = 75e-3;

    // --- Momentum Wheels Configurations --- //
    static inline constexpr double momOfInertiaX = 3.6383e-5; // Moment of inertia for X, [kg.m^2]
    static inline constexpr int16_t maxVelX = 7500;           // Maximum speed for X-momentum wheel, [rpm]

    static inline constexpr double momOfInertiaY = 3.6383e-5; // Moment of inertia for Y, [kg.m^2]
    static inline constexpr int16_t maxVelY = 7500;           // Maximum speed for Y-momentum wheel, [rpm]

    static inline constexpr double momOfInertiaZ = 3.6383e-5; // Moment of inertia for Z, [kg.m^2]
    static inline constexpr int16_t maxVelZ = 7500;           // Maximum speed for Z-momentum wheel, [rpm]

    static inline constexpr double fc = 4.0; // Cut off frequency for angular velocity filtering

    // --- Angular velocity loop gain constants --- //
    static inline constexpr double xVelocityK_p = 0.175;
    static inline constexpr double xVelocityK_i = 1.5e-3 * xVelocityK_p;
    static inline constexpr double xVelocityhLim = 0.5;
    static inline constexpr double xVelocitylLim = -0.5;
    static inline constexpr double xVelocityK_d = 0;

    static inline constexpr double yVelocityK_p = 0.2;
    static inline constexpr double yVelocityK_i = 1.5e-3 * yVelocityK_p;
    static inline constexpr double yVelocityhLim = 1.0;
    static inline constexpr double yVelocitylLim = -1.0;
    static inline constexpr double yVelocityK_d = 0;

    static inline constexpr double zVelocityK_p = 0.2;
    static inline constexpr double zVelocityK_i = 1.5e-3 * zVelocityK_p;
    static inline constexpr double zVelocityhLim = 0.5;
    static inline constexpr double zVelocitylLim = -0.5;
    static inline constexpr double zVelocityK_d = 0;

    // --- Position loop gain constants --- //

    static inline constexpr double xPositionK_p = 0.1;
    static inline constexpr double xPositionK_i = 0 * xPositionK_p;
    static inline constexpr double xPositionhLim = 0.5;
    static inline constexpr double xPositionlLim = -0.5;
    static inline constexpr double xPositionK_d = 0.1;

    static inline constexpr double yPositionK_p = 0.1;
    static inline constexpr double yPositionK_i = 0 * yPositionK_p;
    static inline constexpr double yPositionhLim = 1.0;
    static inline constexpr double yPositionlLim = -1.0;
    static inline constexpr double yPositionK_d = 0.1;

    static inline constexpr double zPositionK_p = 0.2;
    static inline constexpr double zPositionK_i = 0 * zPositionK_p;
    static inline constexpr double zPositionhLim = 0.5;
    static inline constexpr double zPositionlLim = -0.5;
    static inline constexpr double zPositionK_d = 0.1;

    // --- Arbitrary Rotation Configuration --- //

    // Enable automatic find sun operation after detumbling
    static inline constexpr bool enableFindSun = true;

    // Enable arbitrary rotations (if false, no additional rotations after find sun)
    static inline constexpr bool enableArbitraryRotations = true;
    static inline constexpr double vel = 0.02;
    static inline constexpr double accel = 4.0e-3;

    // Queue of rotation commands to execute sequentially
    static inline std::vector<RotationCommand> getRotationQueue()
    {
        return {
            RotationCommand(Vector3d{0.0, 1.0, 0.0}, M_PI / 12.0, vel, accel),  // IN RADIANS
            RotationCommand(Vector3d{0.0, -1.0, 0.0}, M_PI / 12.0, vel, accel), // IN RADIANS
            RotationCommand(Vector3d{1.0, 0.0, 0.0}, M_PI / 12.0, vel, accel)   // IN RADIANS
        };
    }
};
// --- Maxon motor factory functions ---
inline maxon makeXMotor()
{
    maxon m;
    m.nameMotor = "xMomMotor";
    m.serialNo = 0x37058243;
    m.K_P = 259266;
    m.K_I = 2450065;
    m.KFF_VEL = 5306;
    m.KFF_ACC = 6859;
    m.K_TUNING = 1;
    m.KP_CURR = 2000035;
    m.KI_CURR = 3362724;
    m.KT = 24.6e-3;
    m.NUM_OF_PULSE_PER_REV = 512;
    m.SENSOR_POLARITY = 0;
    return m;
}

inline maxon makeYMotor()
{
    maxon m;
    m.nameMotor = "yMomMotor";
    m.serialNo = 0x37058261;
    m.K_P = 260961;
    m.K_I = 2466086;
    m.KFF_VEL = 4699;
    m.KFF_ACC = 6904;
    m.K_TUNING = 1;
    m.KP_CURR = 1822756;
    m.KI_CURR = 2513998;
    m.KT = 24.6e-3;
    m.NUM_OF_PULSE_PER_REV = 512;
    m.SENSOR_POLARITY = 0;
    return m;
}

inline maxon makeZMotor()
{
    maxon m;
    m.nameMotor = "zMomMotor";
    m.serialNo = 0x37059351;
    m.K_P = 266273;
    m.K_I = 2516281;
    m.KFF_VEL = 6600;
    m.KFF_ACC = 7044;
    m.K_TUNING = 1;
    m.KP_CURR = 1789845;
    m.KI_CURR = 2370753;
    m.KT = 24.6e-3;
    m.NUM_OF_PULSE_PER_REV = 512;
    m.SENSOR_POLARITY = 0;
    return m;
}
