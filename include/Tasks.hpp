// Header files for all the tasks
#pragma once
#include "BaseTask.hpp" // BaseTask class declaration
#include <fstream>
#include "Config.hpp"
#include "GrotiferMaster.hpp"

// -----------------------
// Attitude Control
// -----------------------

class AttitudeControl : public BaseTask
{
public:
    AttitudeControl(std::unique_ptr<MaxonMotor> mmX,
                    std::unique_ptr<MaxonMotor> mmY,
                    std::unique_ptr<MaxonMotor> mmZ,
                    std::unique_ptr<ModbusSunSensor> sunSensor,
                    std::unique_ptr<LabJackInclinometer> inclinometer);

    ~AttitudeControl() override;
    int Run() override;

private:
    // States
    static constexpr int INITIALIZING = 0;
    static constexpr int DETERMINING_ATTITUDE = 1;
    static constexpr int INITIALIZING_MOTION = 2;
    static constexpr int DETUMBLING = 3;
    static constexpr int FINDING_SUN = 4;
    static constexpr int HOLDING_POSITION = 5;
    static constexpr int MOVING = 6;

    void InitializeLogs();
    std::ofstream attitudeLog; // holds the body fixed frame
    std::ofstream sensorLog;
    std::ofstream angularVelLog;
    std::ofstream momentumWheelsLog;

    // momentum wheels
    std::unique_ptr<MaxonMotor> p_mmX;
    std::unique_ptr<MaxonMotor> p_mmY;
    std::unique_ptr<MaxonMotor> p_mmZ;

    // sun sensors
    std::unique_ptr<ModbusSunSensor> p_sunSensor;
    std::unique_ptr<LabJackInclinometer> p_inclinometer;

    // max accelerations for momentum wheels
    double maxAccCmdX, maxAccCmdY, maxAccCmdZ;

    // for storing state to calculate angular velocity
    Matrix3d iniRotMat{{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}};
    Matrix3d prevRotMat{{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}};
    double preTime;

    // position and velocity variables
    Vector3d preAngularVelocityVec{{0.0, 0.0, 0.0}}; 
    Vector3d angularVelocityVec{{0.0, 0.0, 0.0}};       // IN BODY FIXED COORDS
    Matrix3d currentOrientation{{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}};
    double rotAngle;

    // for calculating movement profiles
    Vector3d refAngularVelocityVec{{0.0, 0.0, 0.0}};
    double thetaProf, wProf, eProf;
    Vector3d angularVelocityErrorVec{{0.0, 0.0, 0.0}};

    // for the initial kick
    double iniKickEndTime;
    double preTimeInitializingMotion; // Used to track change in time
    bool iniMotionDone = false;

    // for detumbling
    double detumblingEndTime;
    double preTimeDetumbling;
    bool detumblingDone = false;

    // for holding position
    bool holdingPositionSet = false;
    double preTimeHoldingPos;
    Matrix3d holdingPosition{{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}};

    // FOR MOVING PROFILE
    double preTimeMoving;
    double movingProfileAccelerationEndTime;   // accelerate before this time is reached
    double movingProfileConstantEndTime;       // constant velocity before this time is reached
    double movingProfileDecelerationEndTime;   // decelerate before this time is reached (also the end of the move profile)
    bool movingProfileCalculated = false;       // true if the moving profile has been calculated (e. g. the times above have been set)
    bool movingDone = false;
    bool findSunDone = false;
    double refVelocity = 0.015;      // [rad/s] The constant velocity of the moving profile
    double refAcceleration = 2.0e-3; // [rad/s^2] The constant acceleration/deceleration of the moving profile
    Matrix3d startingOrientation{{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}};

    // moving profile variables
    double movingProfileVelocity = 0.0;
    double movingProfileAngle = 0.0;
    double deltaTheta = 0.0;
    Vector3d movingProfileRotAxis{{0.0, 0.0, 0.0}};



    // FOR MOTORS
    int xMomentumWheelVel = 0;
    int yMomentumWheelVel = 0;
    int zMomentumWheelVel = 0;

    // PI control loops (position and velocity)
    PIControl xVelocityLoop,
        yVelocityLoop, zVelocityLoop;

    PIControl xPositionLoop,
        yPositionLoop, zPositionLoop;

    // Named logger instance
    Logger::LoggerInstance logger;

    /**
     * @brief Applies a torque using the momentum wheels
     * @param torque The torque vector to apply
     * @param deltaT The change in time from previous state
     */
    void applyTorque(Vector3d torque, double deltaT);

    /**
     * @brief Moves the X momentum wheel with a given torque
     * @param torque The torque to apply
     * @param deltaT Time over which to apply the torque
     * @param torqueCmdVal Used to extract the final torque commmand
     * @param velCmdVal Used to extract the final velocity command
     * @returns True if the X momentum wheel is saturated
     */
    bool moveXMomentumWheelWithTorque(double torque, double deltaT, double *torqueCmdVal, double *velCmdVal);
    /**
     * @brief Moves the Y momentum wheel with a given torque
     * @param torque The torque to apply
     * @param deltaT Time over which to apply the torque
     * @param torqueCmdVal Used to extract the final torque commmand
     * @param velCmdVal Used to extract the final velocity command
     * @returns True if the X momentum wheel is saturated
     */
    bool moveYMomentumWheelWithTorque(double torque, double deltaT, double *torqueCmdVal, double *velCmdVal);
    /**
     * @brief Moves the Z momentum wheel with a given torque
     * @param torque The torque to apply
     * @param deltaT Time over which to apply the torque
     * @param torqueCmdVal Used to extract the final torque commmand
     * @param velCmdVal Used to extract the final velocity command
     * @returns True if the X momentum wheel is saturated
     */
    bool moveZMomentumWheelWithTorque(double torque, double deltaT, double *torqueCmdVal, double *velCmdVal);
};