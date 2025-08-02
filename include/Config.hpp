#pragma once

#include "GrotiferMaster.hpp"
#include "Logger.hpp"
struct AttitudeConfig
{
    static inline constexpr double deltaTaskTime = 75e-3;

    // --- Momentum Wheels Configurations --- //
    double momOfInertiaX = 3.6383e-5; // Moment of inertia for X, [kg.m^2]
    int16_t maxVelX = 7500;           // Maximum speed for X-momentum wheel, [rpm]

    double momOfInertiaY = 3.6383e-5; // Moment of inertia for Y, [kg.m^2]
    int16_t maxVelY = 7500;           // Maximum speed for Y-momentum wheel, [rpm]

    double momOfInertiaZ = 3.6383e-5; // Moment of inertia for Z, [kg.m^2]
    int16_t maxVelZ = 7500;           // Maximum speed for Z-momentum wheel, [rpm]

    double fc = 4.0; // Cut off frequency for angular velocity filtering

    // Initial Kick Paramters
    double iniKickDuration = 250.0e-3;
    Vector3d iniTorqueVec{{0.0, 0.0, 0.1}};

    // Detumbling Parameters
    double detumblingMaxDuration = 10;

    // --- Flags For Configuring Behavior --- //
    bool initialKick = true; // Perform initial kick to demonstrate detumbling capabilities
    
    // --- Logging Configuration --- //
    Logger::Level logLevel = Logger::Level::DEBUG; 

    // --- Angular velocity loop gain constants --- //
    double xVelocityK_p = 0.175;
    double xVelocityK_i = 1.5e-3 * xVelocityK_p;
    double xVelocityhLim = 0.5;
    double xVelocitylLim = -0.5;
    double xVelocityK_d = 0;

    double yVelocityK_p = 0.2;
    double yVelocityK_i = 1.5e-3 * yVelocityK_p;
    double yVelocityhLim = 1.0;
    double yVelocitylLim = -1.0;
    double yVelocityK_d = 0;

    double zVelocityK_p = 0.2;
    double zVelocityK_i = 1.5e-3 * zVelocityK_p;
    double zVelocityhLim = 0.5;
    double zVelocitylLim = -0.5;
    double zVelocityK_d = 0;

    // --- Position loop gain constants --- //

    double xPositionK_p = 0.1;
    double xPositionK_i = 0 * xPositionK_p;
    double xPositionhLim = 0.5;
    double xPositionlLim = -0.5;
    double xPositionK_d = 0.1;

    double yPositionK_p = 0.1;
    double yPositionK_i = 0 * yPositionK_p;
    double yPositionhLim = 1.0;
    double yPositionlLim = -1.0;
    double yPositionK_d = 0.1;

    double zPositionK_p = 0.05;
    double zPositionK_i = 0 * zPositionK_p;
    double zPositionhLim = 0.5;
    double zPositionlLim = -0.5;
    double zPositionK_d = 0.1;
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
