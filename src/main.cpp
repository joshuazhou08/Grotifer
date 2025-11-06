#include "core/BaseTask.hpp"
#include "tasks/AttitudeControlTask.hpp"
#include "tasks/TorpControlTask.hpp"
#include "Config.hpp"
#include "hardware/actuators/MaxonMotor.hpp"
#include "hardware/actuators/Fan.hpp"
#include "hardware/actuators/StepperMotor.hpp"
#include "hardware/actuators/ThreeAxisActuator.hpp"
#include "hardware/actuators/TorpMaxonActuator.hpp"
#include "hardware/actuators/TorpStepperActuator.hpp"
#include "hardware/sensors/Labjack.hpp"
#include "hardware/sensors/SunSensor.hpp"
#include "hardware/sensors/Inclinometer.hpp"
#include "core/utils/TimeUtils.hpp"
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <iostream>
#include <chrono>
#include <thread>

using namespace std;
using namespace TimeUtils;

#define X_FAN_SERIAL_PORT "/dev/serial/by-id/usb-Pololu_Corporation_Pololu_Jrk_G2_21v3_00464559-if01"
#define Z_FAN_SERIAL_PORT "/dev/serial/by-id/usb-Pololu_Corporation_Pololu_Jrk_G2_21v3_00464472-if01"
#define SUNSENSOR_SERIAL_PORT "/dev/digital_sun_sensor"

// X Momentum Wheel parameter configuration
MaxonParameters makeXMotorParams() {
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
MaxonParameters makeYMotorParams() {
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
MaxonParameters makeZMotorParams() {
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
MaxonParameters makeRightMotorParams() {
    MaxonParameters params;
    params.motorName = "rTorpMotor";
    params.serialNo = 0x40006029;
    params.K_P = 59128;
    params.K_I = 3488178;
    params.KFF_VEL = 888;
    params.KFF_ACC = 0;
    params.K_TUNING = 1.14568;
    params.KP_CURR = 1847321;
    params.KI_CURR = 3129455;
    params.KT = 24.6 * 1.0e-3;
    params.NUM_OF_PULSE_PER_REV = 512;
    params.SENSOR_POLARITY = 0;
    params.homingVel = 0.643;
    params.maxAcc = TorpConfig::accScaleFactor * 0.321;
    params.offsetPos = 244.35 - 0.75;
    params.offsetPosLim = 1.5;
    params.startPosLim = 1.5;
    return params;
}

// Left Maxon Torp parameter configuration
MaxonParameters makeLeftMotorParams() {
    MaxonParameters params;
    params.motorName = "lTorpMotor";
    params.serialNo = 0x40011322;
    params.K_P = 41985;
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
    params.maxAcc = TorpConfig::accScaleFactor * 0.321;
    params.offsetPos = 51.75 - 3.25;
    params.offsetPosLim = 2.5;
    params.startPosLim = 2.0;
    return params;
}

// Left Stepper Motor 1 & 2 configuration
StepperParameters makeL1StepperParams() {
    StepperParameters params;
    params.devicePath = "/dev/left_boom_stepper_1";
    params.baudrate = 9600;
    params.stepMode = 0;
    params.maxSpeed = TorpConfig::stepperSpeed;
    params.startSpeed = TorpConfig::stepperSpeed;
    params.maxCurr = 3500;
    return params;
}

StepperParameters makeL2StepperParams() {
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
StepperParameters makeR1StepperParams() {
    StepperParameters params;
    params.devicePath = "/dev/right_boom_stepper_1";
    params.baudrate = 9600;
    params.stepMode = 0;
    params.maxSpeed = TorpConfig::stepperSpeed;
    params.startSpeed = TorpConfig::stepperSpeed;
    params.maxCurr = 3500;
    return params;
}

StepperParameters makeR2StepperParams() {
    StepperParameters params;
    params.devicePath = "/dev/right_boom_stepper_2";
    params.baudrate = 9600;
    params.stepMode = 0;
    params.maxSpeed = TorpConfig::stepperSpeed;
    params.startSpeed = TorpConfig::stepperSpeed;
    params.maxCurr = 3500;
    return params;
}

void setNonBlockingInput(bool enable)
{
    static termios oldt;
    termios newt;
    if (enable)
    {
        tcgetattr(STDIN_FILENO, &oldt);
        newt = oldt;
        newt.c_lflag &= ~(ICANON | ECHO); // disable buffering and echo
        tcsetattr(STDIN_FILENO, TCSANOW, &newt);
        fcntl(STDIN_FILENO, F_SETFL, O_NONBLOCK); // non-blocking stdin
    }
    else
    {
        tcsetattr(STDIN_FILENO, TCSANOW, &oldt); // restore terminal
        fcntl(STDIN_FILENO, F_SETFL, 0);
    }
}

bool wasEKeyPressed()
{
    char ch;
    if (read(STDIN_FILENO, &ch, 1) > 0)
    {
        return ch == 'e' || ch == 'E';
    }
    return false;
}

int main()
{
    // Initialize the actuators
    int fanBaudRate = 9600;
    Fan xFan(X_FAN_SERIAL_PORT, fanBaudRate, Axis::X);
    if (!xFan.isOpen()) {
        cerr << "[Main] Failed to initialize X Fan" << endl;
        return -1;
    }
    cout << "[Main] X Fan initialized" << endl;
    

    MaxonParameters yMotorParams = makeYMotorParams();
    MaxonMotor yMotor(yMotorParams, Axis::Y);
    
    if (!yMotor.isOpen()) {
        cerr << "[Main] Failed to initialize Y Momentum Wheel" << endl;
        return -1;
    }
    cout << "[Main] Y Momentum Wheel initialized" << endl;

    Fan zFan(Z_FAN_SERIAL_PORT, fanBaudRate, Axis::Z);
    if (!zFan.isOpen()) {
        cerr << "[Main] Failed to initialize Z Fan" << endl;
        return -1;
    }
    cout << "[Main] Z Fan initialized" << endl;
    
    // Torp Control
    MaxonParameters lMotorParams = makeLeftMotorParams();
    MaxonMotor lMotor(lMotorParams, Side::L);

    if (!lMotor.isOpen()) {
        cerr << "[Main] Failed to initialize L Torp Arm Motor" << endl;
        return -1;
    }
    cout << "[Main] L Torp Arm Motor initialized" << endl;

    MaxonParameters rMotorParams = makeRightMotorParams();
    MaxonMotor rMotor(rMotorParams, Side::R);

    if (!rMotor.isOpen()) {
        cerr << "[Main] Failed to initialize R Torp Arm Motor" << endl;
        return -1;
    }
    cout << "[Main] R Torp Arm Motor initialized" << endl;

    StepperParameters StepperL1Params = makeL1StepperParams();
    StepperMotor L1Motor(StepperL1Params, StepperMot::L1);

    if (!L1Motor.isOpen()) {
        cerr << "[Main] Failed to initialize L1 Stepper Motor" << endl;
        return -1;
    }
    cout << "[Main] L1 Stepper Motor initialized" << endl;

    StepperParameters StepperL2Params = makeL2StepperParams();
    StepperMotor L2Motor(StepperL2Params, StepperMot::L2);

    if (!L2Motor.isOpen()) {
        cerr << "[Main] Failed to initialize L2 Stepper Motor" << endl;
        return -1;
    }
    cout << "[Main] L2 Stepper Motor initialized" << endl;

    StepperParameters StepperR1Params = makeR1StepperParams();
    StepperMotor R1Motor(StepperR1Params, StepperMot::R1);

    if (!R1Motor.isOpen()) {
        cerr << "[Main] Failed to initialize R1 Stepper Motor" << endl;
        return -1;
    }
    cout << "[Main] R1 Stepper Motor initialized" << endl;

    StepperParameters StepperR2Params = makeR2StepperParams();
    StepperMotor R2Motor(StepperR2Params, StepperMot::R2);

    if (!R2Motor.isOpen()) {
        cerr << "[Main] Failed to intialize R2 Stepper Motor" << endl;
        return -1;
    }
    cout << "[Main] R2 Stepper Motor initialized" << endl;


    // Create three-axis actuator system
    ThreeAxisActuator threeAxisActuator(xFan, yMotor, zFan);
    cout << "[Main] ThreeAxisActuator created with X Fan, Y Wheel, Z Fan" << endl;

    // Create Maxon torp actuator system
    TorpMaxonActuator torpMaxonActuator(lMotor, rMotor);
    cout << "[Main] TorpMaxonActuator created with L and R Maxon Motors" << endl;

    // Create Stepper torp actuator system
    TorpStepperActuator torpStepperActuator(L1Motor, L2Motor, R1Motor, R2Motor);
    cout << "[Main] TorpStepperActuator created with L1, L2, R1, R2 Stepper Motors" << endl;

    // Initialize the sensors and labjack
    int pinOffset = 0;
    LabJackU6 labJack(pinOffset);
    if (!labJack.isOpen()) {
        cerr << "[Main] Failed to initialize LabJack" << endl;
        return -1;
    }
    cout << "[Main] LabJack initialized" << endl;

    int encoderCPR = 400;
    LJEncoder rightEnc(labJack, encoderCPR, 1);
    LJEncoder leftEnc(labJack, encoderCPR, 2);

    int baudRate = 115200;
    SunSensor sunSensor(SUNSENSOR_SERIAL_PORT, baudRate);
    if (!sunSensor.isOpen()) {
        cerr << "[Main] Failed to initialize Sun Sensor" << endl;
        return -1;
    }
    cout << "[Main] Sun Sensor initialized" << endl;

    int xChannel = 0;
    int yChannel = 1;
    Inclinometer inclinometer(labJack, xChannel, yChannel);
    if (!inclinometer.isOpen()) {
        cerr << "[Main] Failed to initialize Inclinometer" << endl;
        return -1;
    }
    cout << "[Main] Inclinometer initialized" << endl;

    // Sleep to let us read the device initialize logs
    this_thread::sleep_for(chrono::seconds(3));

    // Initialize control loops
    ControlLoops controlLoops{
        .xVelocityLoop = PIControl(
            AttitudeConfig::xVelocityK_p,
            AttitudeConfig::xVelocityK_i,
            AttitudeConfig::xVelocityhLim,
            AttitudeConfig::xVelocitylLim,
            AttitudeConfig::xVelocityK_d),
        .yVelocityLoop = PIControl(
            AttitudeConfig::yVelocityK_p,
            AttitudeConfig::yVelocityK_i,
            AttitudeConfig::yVelocityhLim,
            AttitudeConfig::yVelocitylLim,
            AttitudeConfig::yVelocityK_d),
        .zVelocityLoop = PIControl(
            AttitudeConfig::zVelocityK_p,
            AttitudeConfig::zVelocityK_i,
            AttitudeConfig::zVelocityhLim,
            AttitudeConfig::zVelocitylLim,
            AttitudeConfig::zVelocityK_d),
        .xPositionLoop = PIControl(
            AttitudeConfig::xPositionK_p,
            AttitudeConfig::xPositionK_i,
            AttitudeConfig::xPositionhLim,
            AttitudeConfig::xPositionlLim,
            AttitudeConfig::xPositionK_d),
        .yPositionLoop = PIControl(
            AttitudeConfig::yPositionK_p,
            AttitudeConfig::yPositionK_i,
            AttitudeConfig::yPositionhLim,
            AttitudeConfig::yPositionlLim,
            AttitudeConfig::yPositionK_d),
        .zPositionLoop = PIControl(
            AttitudeConfig::zPositionK_p,
            AttitudeConfig::zPositionK_i,
            AttitudeConfig::zPositionhLim,
            AttitudeConfig::zPositionlLim,
            AttitudeConfig::zPositionK_d)
    };

    AttitudeControl attitudeControl(
        threeAxisActuator,
        sunSensor,
        inclinometer,
        controlLoops);

    // Initialize PI for torps
    TorpPIControl torpPIControl{
        .leftMotor = PIControl(
            TorpConfig::l_kp,
            TorpConfig::l_ki,
            TorpConfig::l_hLim,
            TorpConfig::l_lLim,
            TorpConfig::l_kd),
        .rightMotor = PIControl(
            TorpConfig::r_kp,
            TorpConfig::r_ki,
            TorpConfig::r_hLim,
            TorpConfig::r_lLim,
            TorpConfig::r_kd)
    };
    TorpControl rightTorp(
        torpMaxonActuator,
        torpStepperActuator,
        torpPIControl,
        labJack,
        rightEnc,
        Side::R
    );

    TorpControl leftTorp(
        torpMaxonActuator,
        torpStepperActuator,
        torpPIControl,
        labJack,
        leftEnc,
        Side::L
    );

    // Initialize Task List
    constexpr int NUM_TASKS = 1;

    BaseTask *taskTable[NUM_TASKS] = {
        &attitudeControl};

    double startTime = GetTimeNow();
    double T_PROGRAM = 60;
    int i = 0;

    setNonBlockingInput(true); // Enable non-blocking input for smooth kill signal
    while (GetTimeNow() - startTime <= T_PROGRAM)
    {
        if (wasEKeyPressed())
        {
            cout << "[Main] Kill signal detected (E pressed)" << endl;
            break;
        }
        taskTable[i]->Run();
        i = (i + 1) % NUM_TASKS;
    }
    setNonBlockingInput(false); // Restore terminal settings
}
