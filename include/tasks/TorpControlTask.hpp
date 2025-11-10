#pragma once
#include "core/BaseTask.hpp" // BaseTask class declaration
#include "core/control/PIControl.hpp"
#include "hardware/actuators/MaxonMotor.hpp"
#include "hardware/actuators/TorpMaxonActuator.hpp"
#include "hardware/actuators/TorpStepperActuator.hpp"
#include "hardware/sensors/Sensors.hpp"
#include "hardware/sensors/Labjack.hpp"
#include "core/solvers/MovingAverage.hpp"
#include "Config.hpp"
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <filesystem>
#include <cmath>
#include <future>
#include <chrono>


// PI Control for Torp Maxon Motors
struct TorpPIControl {
    PIControl leftMotor;
    PIControl rightMotor;
};

// Torp Control States enum
enum TorpControlState
{
    INITIALIZING_TORP = 0,
    FINDING_HOME_INDEX = 1,
    MOVING_TO_START_POS = 2,
    WAITING_FOR_SYNC = 3,
    ACCELERATING = 4,
    CRUISING = 5,
    DEPLOYING_MASS = 6,
    RETRACTING_MASS = 7,
    DECELERATING = 8,
    STOPPING = 9
};

// -----------------------
// Torp Control
// -----------------------

class TorpControl : public BaseTask
{
public:
    explicit TorpControl(
                TorpMaxonActuator& torpMaxonActuator,
                TorpStepperActuator& torpStepperActuator,
                TorpPIControl& torpPIControl,
                LabJackU6& labjack,
                LJEncoder& encoder,
                Side side);
    
    ~TorpControl() override;
    int Run() override;

    const double PI = M_PI;
    const double CONV_RAD_TO_DEG = 180 / PI;
    const double CONV_RPM_TO_RADpSEC = PI / 30;

    // Setters
    inline void enable() { enabled = true; };
    
    // Task coordinator ends torp sequence after moves
    inline void spinDown() { 

       startSpinningDownFlag = true;
       
    };

    // Getters
    inline bool doneSpinningUp() { return torpCruisingFlag; }; 
    inline bool isEnabled() { return enabled; };
private:

    // Maxon and Stepper Motor actuator systems
    TorpMaxonActuator& torpMaxonActuator_;
    TorpStepperActuator& torpStepperActuator_;

    // PI Control
    TorpPIControl& torpPIControl_;
    PIControl& leftMotor;
    PIControl& rightMotor;

    // Handle to pick motor for PI calculation
    PIControl* pi_ = nullptr;

    // Sensors
    LabJackU6& labjack_;
    LJEncoder& encoder_;

    Side side_;

    // == Torp Control Variables == //

    // Torp Position and Velocity variables 
    double torpPos,                // Torp arm position retrieved from labjack
           torpPrePos,             // Starting position from labjack used in torpVel calculation
           torpVel,                // Torp velocity calculated using finite difference (RPM)
           motPos,                 // Motor position retrieved from Maxon controller
           motVel,                 // Motor speed retrieved from Maxon controller
           posErr;                 // Position error of torp arms pulled from PI controller
    
    // Reference profile variables for logging
    double refPos,                 // Home position, or calculated from refVel integration
           refAcc,                 // Commanded acceleration for torp arms at any time (RPM/s)
           refVel,                 // Commanded velocity for torp arms at any time (RPM)
           desVel;                 // Calculated velocity with PI Controller

    // Home/start position variables for logging
    double homeTorpPos,            // Home position for torp retrieved from labjack at index
           homeMotPos,             // Home position for motot retrieved from Maxon controller at index
           startPosRef,            // Target starting position offset from home position
           startPosAct;            // Actual starting position where the torp arms stop

    // Homing velocity/acceleration variables
    double homingVel,              // Reference homing velocity for Maxons
           maxAcc,                 // Reference max acceleration for Maxons
           accMag,                 // Reference acceleration during deceleration of torps
           offsetPos,              // Offset distance from home index to starting position (deg)
           offsetPosLim,           // Minimum offset distance that requires deceleration profile (deg)
           startPosLim;            // Position error tolerance for reaching starting position

    // Variable to ensure velocity signal is always positive
    bool flipSign;                 // CCW (+) or CW(-), used to ensure correct Maxon movement

    // PI timing variables
    double time,                   // Set at the beginning of every cycle, used for timing comparisons
           deltaT;                 // Current time - preTime, used in velocity integration

    // Synchronization timing variables
    double preTime,                // Time before current cycle starts, used to calculate deltaT
           timeStart,              // Time at beginning of cycle
           timeEnd;                // Time at end of cycle

    // Velocity profile timing change variables
    double tA,                     // Start time for acceleration/deceleration phase
           tB;                     // Calculated end time for acceleration/deceleration phase

    // Flags for transitions
    bool indexFlag,                // Flag to identify if torp has reached index position
         startPosLocFlag,          // Used when torps are approaching starting position
         doneHomingFlag,           // Both torp arms have finished homing
         torpCruisingFlag,         // Torp sequence has spun up to cruise and deployed masses -- out to task coordinator
         startSpinningDownFlag;    // Moves have completed, torp retract masses and spin down -- in from task coordinator

    static bool leftHomingFlag,           // Left torp has finished homing
                rightHomingFlag;          // Right torp arm has finished homing

    // == TorpMasterControl variables == //
    // Deployment/Retraction flags
    bool deployStartFlag,          // Cruising started, masses have started to deploy
         retractStartFlag,         // Cruising ended, masses have started to retract
         deployDoneFlag,           // Used when masses finish deploying
         retractDoneFlag,          // Used when masses finish retracting
         readyToDeployFlag,        // Used when acceleration has completed, masses ready to deploy
         firstRunFlag = true;      // Used on first cycle to set initial preTime

    // Acceleration, deployment, cruising variables
    double oprVelMag,              // Operating velocity during deployement (RPM)
           maxAccAllowed,          // Max acceleration as calculated for deployment state (rpm/s)
           tAccDec,                // Time for acceleration/deceleration for deployment state (s)
           tCruise,                // Cruise time for deployment phase (s)
           tDeployRetract,         // Time for deploying/retracting (s)
           tEndDeployRetract;      // Calculated time at end of deploy/retract (s)
    
    // S-Curve Acceleration profile variables
    double posProfVal,             // Position durign S-Curve motion profile
           accProfVal,             // Acceleration during S-Curve motion profile
           velProfVal,             // Velocity during S-Curve motion profile
           jerkProfVal;            // Rate of change of accel/decel during S-curve motion profile (rpm/s^2)

    double Ta,                     // Time at the end of acceleration/deceleration ramp-up
           Tb,                     // Time at the beginning of acceleration/deceleration ramp-down
           Tc,                     // Time when acceleration/deceleration is finished
           Td,                     // Time for each phase of cruising state
           T1,                     // Ramp-up and ramp-down time duration
           T2;                     // Constant acceleration time duration

    static bool enabled;           // Flag to start torp sequence from task coordinator
    
    int movingAverageFilterSpan = 5;
    MovingAverage velMAFilter{movingAverageFilterSpan};
    
    // ====== TORP CONTROL FUNCTIONS ====== //

    // Getter functions for encoder readings used in Torp Control

    /**
     * @brief Returns angular position from labjack encoder (deg)
     */
    inline double getAngularPosDeg() { return encoder_.getAngularPosDeg(); }

    /**
     * @brief Returns flag as true when encoder reaches the index position
     */
    inline bool getIndexFlag() { return encoder_.getIndexFlag(); }

    // Getter functions for Torp homing related values
    // ==== are any of these needed? ====== // 
    
    double getRefPosition() { return refPos; }
    double getRefVelocity() { return refVel; }
    double getRefAcc() {return refAcc;}
    double getHomeTorpPos() {return homeTorpPos;}
    double getHomeMotPos() {return homeMotPos;}
    double getStartingPosAct() {return startPosAct;}
    double getStartingPosRef() {return startPosRef;}
    double getIndexFlagValue() {return indexFlag;}
    double getMotorPos() {return motPos;}
    double getMotorVel() {return motVel;}
    double getPosError() {return posErr;}
    double getTorpPos() {return torpPos;}
    double getTorpVel() {return torpVel;}
    double getDesVel() {return desVel;}

    // Getter functions for Torp master control
    
    double getAccProfVal() {return accProfVal;}
    double getVelProfVal() {return velProfVal;}
    double getJerkProfVal() {return jerkProfVal;}
    double getPosProfVal() {return posProfVal;}
    bool getDeployingDoneFlag() {return deployDoneFlag;}
    bool getRetractingDoneFlag() {return retractDoneFlag;}
    bool getReadyToDeploySensorsFlag() {return readyToDeployFlag;}
    bool getFirstTimeRunningFlag() {return firstRunFlag;}

    // == Functions for Torp Control == //

    /**
     * @brief Function ensures that motor is spinning in desired direction
     * @param flipSignFlag
     */
    
    int getSignDir(bool flipSignFlag);

    /**
     * @brief Function to round the desired velocity for command to Maxon Motors
     * @param desVel * @param gearRatio -- calculated desired velocity to send to motors
     */
    int roundingFunc(double val);

};