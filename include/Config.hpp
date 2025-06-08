#pragma once

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
};
