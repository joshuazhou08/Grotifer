// Header files for all the tasks
#pragma once
#include "BaseTask.hpp" // BaseTask class declaration
#include <fstream>
#include <queue>
#include "Config.hpp"
#include "GrotiferMaster.hpp"

// -----------------------
// Attitude Control
// -----------------------

class AttitudeControl : public BaseTask
{
public:
    AttitudeControl(ThreeAxisActuator& threeAxisActuator,
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
    static constexpr int HOLDING_POSITION = 4;
    static constexpr int MOVING = 5;

    void InitializeLogs();
    std::ofstream attitudeLog; // holds the body fixed frame
    std::ofstream sensorLog;
    std::ofstream angularVelLog;
    std::ofstream momentumWheelsLog;

    // Three-axis actuator system (fans and momentum wheels)
    ThreeAxisActuator& threeAxisActuator_;

    // Sensors
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
    Vector3d angularVelocityVec{{0.0, 0.0, 0.0}}; // IN BODY FIXED COORDS
    Matrix3d currentOrientation{{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}};
    double rotAngle;

    // for calculating movement profiles
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
    double movingProfileAccelerationEndTime; // accelerate before this time is reached
    double movingProfileConstantEndTime;     // constant velocity before this time is reached
    double movingProfileDecelerationEndTime; // decelerate before this time is reached (also the end of the move profile)
    bool movingProfileCalculated = false;    // true if the moving profile has been calculated (e. g. the times above have been set)
    bool movingDone = false;
    double refVelocity = 0.02;       // [rad/s] The constant velocity of the moving profile
    double refAcceleration = 4.0e-3; // [rad/s^2] The constant acceleration/deceleration of the moving profile
    Matrix3d startingOrientation{{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}};

    // moving profile variables
    double movingProfileVelocity = 0.0;
    double movingProfileAngle = 0.0;
    double deltaTheta = 0.0;
    Vector3d movingProfileRotAxis{{0.0, 0.0, 0.0}};

    // Rotation queue management
    std::queue<RotationCommand> rotationQueue;
    RotationCommand currentRotationCommand{{0.0, 0.0, 1.0}, 0.0}; // Default command
    bool rotationQueueInitialized = false;

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
     * @brief Sets the holding position and configures the right flags
     * @param position The position to hold
     */
    void setHoldingPosition(Matrix3d position);

    /**
     * @brief Calculates and prepends the find sun rotation command to the front of the queue
     * This calculates the rotation needed to go from current orientation to identity matrix
     */
    void prependFindSunRotation();

    /**
     * @brief uses a cascaded controller to output the torque signal to follow a target
     * @param target Where you want to be
     * @param current Where you currently are
     * @return torque to apply
     */
    Vector3d cascadeControl(Matrix3d target, Matrix3d current, Vector3d refAngularVelocityVec);
};