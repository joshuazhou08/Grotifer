// Header files for all the tasks
#pragma once
#include "core/BaseTask.hpp" // BaseTask class declaration
#include "core/control/PIControl.hpp"
#include <fstream>
#include <queue>
#include <Eigen/Dense>
#include "Config.hpp"
#include "hardware/sensors/Sensors.hpp"
#include "hardware/actuators/ThreeAxisActuator.hpp"

using Eigen::Matrix3d;
using Eigen::Vector3d;

// Control loops configuration struct
struct ControlLoops {
    PIControl xVelocityLoop;
    PIControl yVelocityLoop;
    PIControl zVelocityLoop;
    PIControl xPositionLoop;
    PIControl yPositionLoop;
    PIControl zPositionLoop;
};

// Attitude Control state enum
enum AttitudeControlState
{
    INITIALIZING = 0,
    DETERMINING_ATTITUDE = 1,
    INITIALIZING_MOTION = 2,
    DETUMBLING = 3,
    HOLDING_POSITION = 4,
    MOVING = 5
};

// -----------------------
// Attitude Control
// -----------------------

class AttitudeControl : public BaseTask
{
public:
    AttitudeControl(ThreeAxisActuator& threeAxisActuator,
                    Sensor& sunSensor,
                    Sensor& inclinometer,
                    ControlLoops& controlLoops);

    ~AttitudeControl() override;
    int Run() override;

private:

    // Three-axis actuator system (fans and momentum wheels)
    ThreeAxisActuator& threeAxisActuator_;

    // Sensors
    Sensor& sunSensor_;
    Sensor& inclinometer_;


    // for storing state to calculate angular velocity
    Matrix3d prevOrientation{{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}};
    double preTime;
    Vector3d preAngularVelocityVec{{0.0, 0.0, 0.0}};

    // for calculating movement profiles
    double thetaProf, wProf, eProf;
    Vector3d angularVelocityErrorVec{{0.0, 0.0, 0.0}};

    // for the initial kick
    double iniKickEndTime;

    // for detumbling
    double detumblingEndTime;

    // for holding position
    bool holdingPositionSet = false;
    Matrix3d holdingPosition{{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}};

    // FOR MOVING PROFILE
    double movingProfileAccelerationEndTime; // accelerate before this time is reached
    double movingProfileConstantEndTime;     // constant velocity before this time is reached
    double movingProfileDecelerationEndTime; // decelerate before this time is reached (also the end of the move profile)
    bool movingProfileCalculated = false;    // true if the moving profile has been calculated (e. g. the times above have been set)
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
    
    // PI control loops (position and velocity)
    PIControl xVelocityLoop;
    PIControl yVelocityLoop;
    PIControl zVelocityLoop;
    PIControl xPositionLoop;
    PIControl yPositionLoop;
    PIControl zPositionLoop;

    /**
     * @brief Applies torque with sign correction for Y and Z axes
     * @param torque The torque vector to apply
     * @param deltaT The change in time from previous state
     */
    void applyTorque(Vector3d torque, double deltaT);

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

    /**
     * @brief Initializes the moving profile for the next rotation in the queue
     * @param currentOrientation The current orientation matrix
     */
    void initializeMovingProfile(const Matrix3d& currentOrientation);

    /**
     * @brief Calculates the motion profile velocity and orientation at the current time
     * @param time Current time
     * @param deltaT Time since last iteration
     * @return Pair of (inertial frame angular velocity vector, target orientation matrix)
     */
    std::pair<Vector3d, Matrix3d> calculateMotionProfile(double time, double deltaT);
};