#pragma once

#include "GrotiferMaster.hpp"
struct AttitudeConfig
{
    static inline constexpr double deltaTaskTime = 75e-3;

    // --- Momentum Wheels Configurations --- //
    // X-momentum wheel
    double momOfInertiaX = 3.6383e-5; // Moment of inertia for X, [kg.m^2]
    long int maxVelX = 4600;          // Maximum speed for X-momentum wheel, [rpm]

    // Y-momentum wheel
    double momOfInertiaY = 3.6383e-5; // Moment of inertia for Y, [kg.m^2]
    long int maxVelY = 4600;          // Maximum speed for Y-momentum wheel, [rpm]

    // Z-momentum wheel
    double momOfInertiaZ = 3.6383e-5; // Moment of inertia for Z, [kg.m^2]
    long int maxVelZ = 4600;          // Maximum speed for Z-momentum wheel, [rpm]

    double fc = 4.0; // Cut off frequency for angular velocity filtering
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
