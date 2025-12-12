#pragma once
#include "hardware/actuators/MaxonMotor.hpp"
#include "hardware/actuators/StepperMotor.hpp"
#include "Config.hpp"
// X Momentum Wheel parameter configuration
inline MaxonParameters makeXMotorParams() {
    MaxonParameters params;
    params.motorName = "xMomMotor";
    params.serialNo = 0x37058243;
    params.K_P = 259266;
    params.K_I = 2450065;
    params.KFF_VEL = 5306;
    params.KFF_ACC = 6859;
    params.K_TUNING = 1;
    params.KP_CURR = 2000035;
    params.KI_CURR = 3362724;
    params.KT = 24.6e-3;
    params.NUM_OF_PULSE_PER_REV = 512;
    params.SENSOR_POLARITY = 0;
    params.momentOfInertia = 3.6383e-5; // [kg*m^2]
    params.maxVelocity = 7500;          // [rpm]
    return params;
}

// Y Momentum Wheel parameter configuration
inline MaxonParameters makeYMotorParams() {
    MaxonParameters params;
    params.motorName = "yMomMotor";
    params.serialNo = 0x37058261;
    params.K_P = 260961;
    params.K_I = 2466086;
    params.KFF_VEL = 4699;
    params.KFF_ACC = 6904;
    params.K_TUNING = 1;
    params.KP_CURR = 1822756;
    params.KI_CURR = 2513998;
    params.KT = 24.6e-3;
    params.NUM_OF_PULSE_PER_REV = 512;
    params.SENSOR_POLARITY = 0;
    params.momentOfInertia = 3.6383e-5; // [kg*m^2]
    params.maxVelocity = 7500;          // [rpm]
    return params;
}

// Z Momentum Wheel parameter configuration
inline MaxonParameters makeZMotorParams() {
    MaxonParameters params;
    params.motorName = "zMomMotor";
    params.serialNo = 0x37059351;
    params.K_P = 266273;
    params.K_I = 2516281;
    params.KFF_VEL = 6600;
    params.KFF_ACC = 7044;
    params.K_TUNING = 1;
    params.KP_CURR = 1789845;
    params.KI_CURR = 2370753;
    params.KT = 24.6e-3;
    params.NUM_OF_PULSE_PER_REV = 512;
    params.SENSOR_POLARITY = 0;
    params.momentOfInertia = 3.6383e-5; // [kg*m^2]
    params.maxVelocity = 7500;          // [rpm]
    return params;
}

// Right Maxon Torp parameter configuration
inline MaxonParameters makeRightMotorParams() {
    MaxonParameters params;
    params.motorName = "rTorpMotor";
    params.serialNo = 0x40006029;
    params.K_P = 44366;
    params.K_I = 3488178;
    params.KFF_VEL = 888;
    params.KFF_ACC = 0;
    params.K_TUNING = 1.14568;
    params.KP_CURR = 1847321;
    params.KI_CURR = 3129455;
    params.KT = 24.6 * 1.0e-3;
    params.NUM_OF_PULSE_PER_REV = 512;
    params.SENSOR_POLARITY = 0;
    params.homingVel = -1 * 0.643;
    params.maxAcc = TorpConfig::acc;
    params.offsetPos = 244.35 - 0.75;
    params.offsetPosLim = 1.5;
    params.startPosLim = 1.5;
    return params;
}

// Left Maxon Torp parameter configuration
inline MaxonParameters makeLeftMotorParams() {
    MaxonParameters params;
    params.motorName = "lTorpMotor";
    params.serialNo = 0x40011322;
    params.K_P = 31488;
    params.K_I = 2219421;
    params.KFF_VEL = 2058;
    params.KFF_ACC = 397;
    params.K_TUNING = 1.5013;
    params.KP_CURR = 1171880;
    params.KI_CURR = 3906250;
    params.KT = 24.6 * 1.0e-3;
    params.NUM_OF_PULSE_PER_REV = 512;
    params.SENSOR_POLARITY = 0;
    params.homingVel = 0.643;
    params.maxAcc = TorpConfig::acc;
    params.offsetPos = 51.75 - 3.25;
    params.offsetPosLim = 2.5;
    params.startPosLim = 2.0;
    return params;
}

// Left Stepper Motor 1 & 2 configuration
inline StepperParameters makeL1StepperParams() {
    StepperParameters params;
    params.devicePath = "/dev/left_boom_stepper_1";
    params.baudrate = 9600;
    params.stepMode = 0;
    params.maxSpeed = TorpConfig::stepperSpeed;
    params.startSpeed = TorpConfig::stepperSpeed;
    params.maxCurr = 3500;
    return params;
}

inline StepperParameters makeL2StepperParams() {
    StepperParameters params;
    params.devicePath = "/dev/left_boom_stepper_2";
    params.baudrate = 9600;
    params.stepMode = 0;
    params.maxSpeed = TorpConfig::stepperSpeed;
    params.startSpeed = TorpConfig::stepperSpeed;
    params.maxCurr = 3500;
    return params;
}

// Right Stepper Motor 1 & 2 configuration
inline StepperParameters makeR1StepperParams() {
    StepperParameters params;
    params.devicePath = "/dev/right_boom_stepper_1";
    params.baudrate = 9600;
    params.stepMode = 0;
    params.maxSpeed = TorpConfig::stepperSpeed;
    params.startSpeed = TorpConfig::stepperSpeed;
    params.maxCurr = 3500;
    return params;
}

inline StepperParameters makeR2StepperParams() {
    StepperParameters params;
    params.devicePath = "/dev/right_boom_stepper_2";
    params.baudrate = 9600;
    params.stepMode = 0;
    params.maxSpeed = TorpConfig::stepperSpeed;
    params.startSpeed = TorpConfig::stepperSpeed;
    params.maxCurr = 3500;
    return params;
}