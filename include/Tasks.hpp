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
    // Configs
    AttitudeConfig config;
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
    Vector3d angularVelocityVec{{0.0, 0.0, 0.0}};
    Matrix3d solRotMatBackward{{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}};
    double rotAngle;

    // for calculating movement profiles
    Vector3d refAngularVelocityVec{{0.0, 0.0, 0.0}};
    double thetaProf, wProf, eProf;
    Vector3d angularVelocityErrorVec{{0.0, 0.0, 0.0}};

    // for the initial kick
    double iniKickEndTime;
    double preTimeIni; // Used to track change in time
    bool iniMotionDone = false;

    // for motors
    int xMomentumWheelVel = 0;
    int yMomentumWheelVel = 0;
    int zMomentumWheelVel = 0;

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