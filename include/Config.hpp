#pragma once

#include "GrotiferMaster.hpp"
#include "Devices.hpp"
#include "SoftwareFunctions.hpp"

#define RIGHT_STEPPER_1 "/dev/right_boom_stepper_1"
#define RIGHT_STEPPER_2 "/dev/right_boom_stepper_2"
#define LEFT_STEPPER_1 "/dev/left_boom_stepper_1"
#define LEFT_STEPPER_2 "/dev/left_boom_stepper_2"

// ====== ATTITUDE CONTROL ====== //

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

// ====== TORP CONTROL ======= //

// --- Torp config --- //
struct TorpConfig
{
    // ********* deltaTaskTime defined somewhere better than here and below??  *********
    static inline constexpr double deltaTaskTime = 150.0e-3;
    static inline constexpr double gearRatio = 31.1;

    static inline constexpr double oprVel = 12; // operation velocity (rpm)
    static inline constexpr double accScaleFactor = 1.0; // scale factor for torp max acceleration, 1.0 for mounted weights
    static inline constexpr double maxAccMag = accScaleFactor * 0.321; // max acceleration (rpm/s)
    static inline constexpr double tAccDec = 60; // acc/dec time (s), use 60 for mounted weights
    static inline constexpr double tCruise = 20; // cruising time (s)
    static inline constexpr double homingVelMag = 0.643; // homing velocity of the torps
    static inline constexpr double stepperSpeed = (uint32_t) ((200.0 / 60) * (1.034 / 0.04)); // stepper motor speed (steps/10000s)

    // other variables
};

struct TaskCoordinateConfig
{
    static inline constexpr double deltaTaskTime = 150.0e-3;
    static inline constexpr bool holdPosAfterDeployFlag = true;
    static inline constexpr bool sensorsOnlyFlag = false;
    static inline constexpr bool controlBodyOnlyFlag = false;
    static inline constexpr bool controlBodyNoFindSunFlag = false;
};

// --- Maxon motor factory functions --- //

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

inline maxon makeLeftMotor()
{
    maxon m;
    m.nameMotor = "LeftTORPMOT";
    m.serialNo = 0x40011322;
    m.K_P = 41985;
    m.K_I = 2219421;
    m.KFF_VEL = 2058;
    m.KFF_ACC = 397;
    m.K_TUNING = 1.5013;
    m.KP_CURR = 1171880;
    m.KI_CURR = 3906250;
    m.KT = 24.6 * 1.0e-3;
    m.NUM_OF_PULSE_PER_REV = 512;
    m.SENSOR_POLARITY = 0;
}

inline maxon makeRightMotor()
{
    maxon m;
    m.nameMotor = "RightTORPMOT";
    m.serialNo = 0x40006029;
    m.K_P = 59128;
    m.K_I = 3488178;
    m.KFF_VEL = 888;
    m.KFF_ACC = 0;
    m.K_TUNING = 1.14568;
    m.KP_CURR = 1847321;
    m.KI_CURR = 3129455;
    m.KT = 24.6 * 1.0e-3;
    m.NUM_OF_PULSE_PER_REV = 512;
    m.SENSOR_POLARITY = 0;   
}

// --- PI torp motion controller --- //

// declared in SoftwareFunctions.hpp
inline PIControlPara leftTorpPI()
{
    PIControlPara PIctrl;
    PIctrl.kp = 10;
    PIctrl.ki = 8.5;
    PIctrl.hLim = 32.0;
    PIctrl.lLim = -32.0;
}

inline PIControlPara rightTorpPI()
{
    PIControlPara PIctrl;
    PIctrl.kp = 10;
    PIctrl.ki = 8.5;
    PIctrl.hLim = 32.0;
    PIctrl.lLim = -32.0;
}

// --- Torp homing profile functions --- //

// declared in SoftwareFunctions.hpp
inline homingProfilePara leftHomingProf()
{
    homingProfilePara homing;
    homing.homingVel = TorpConfig::homingVelMag;
    homing.maxAcc = TorpConfig::maxAccMag;
    homing.offsetPos = 51.75 - 3.25;
    homing.offsetPosLim = 2.5;
    homing.startPosLim = 2.0;
}

inline homingProfilePara rightHomingProf()
{
    homingProfilePara homing;
    homing.homingVel = TorpConfig::homingVelMag;
    homing.maxAcc = TorpConfig::maxAccMag;
    homing.offsetPos = 244.35 - 0.75;
    homing.offsetPosLim = 1.5;
    homing.startPosLim = 1.5;
}

// --- Stepper motor parameter functions --- //

// declared in Actuators.hpp
inline stepperPara leftStepper1Params()
{
    stepperPara step;
    step.sm = open(LEFT_STEPPER_1, O_RDWR | O_NOCTTY);
    step.baudrate = 9600;
    step.maxCurr = 3500;
    step.stepMode = 0;
    step.maxSpeed = TorpConfig::stepperSpeed;
    step.startSpeed = TorpConfig::stepperSpeed;
}

inline stepperPara leftStepper2Params()
{
    stepperPara step;
    step.sm = open(LEFT_STEPPER_2, O_RDWR | O_NOCTTY);
    step.baudrate = 9600;
    step.maxCurr = 3500;
    step.stepMode = 0;
    step.maxSpeed = TorpConfig::stepperSpeed;
    step.startSpeed = TorpConfig::stepperSpeed;
}

inline stepperPara rightStepper1Params()
{
    stepperPara step;
    step.sm = open(RIGHT_STEPPER_1, O_RDWR | O_NOCTTY);
    step.baudrate = 9600;
    step.maxCurr = 3500;
    step.stepMode = 0;
    step.maxSpeed = TorpConfig::stepperSpeed;
    step.startSpeed = TorpConfig::stepperSpeed;
}

inline stepperPara rightStepper2Params()
{
    stepperPara step;
    step.sm = open(RIGHT_STEPPER_2, O_RDWR | O_NOCTTY);
    step.baudrate = 9600;
    step.maxCurr = 3500;
    step.stepMode = 0;
    step.maxSpeed = TorpConfig::stepperSpeed;
    step.startSpeed = TorpConfig::stepperSpeed;
}

// encoder?? 

    
