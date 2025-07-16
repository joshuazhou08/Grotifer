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
};

class TorpControl : public BaseTask
{
public:
    TorpControl(homingProfilePara homingPara,
                std::unique_ptr<PIControl> p_pi,
                std::unique_ptr<MaxonMotor> p_mm,
                std::unique_ptr<LJEncoder3Channels> p_ljEnc3C,
                double gearRatio);
    ~TorpControl() override;
    int Run() override;

    // functions for logging
    double GetRefPosition() {return refPos;}
    double GetRefVelocity() {return refVel;}
    double GetRefAcc() {return refAcc;}
    double GetHomeTorpPos() {return homeTorpPos;}
    double GetHomeMotPos() {return homeMotPos;}
    double GetStartingPosAct() {return startPosAct;}
    double GetStartingPosRef() {return startPosRef;}
    double GetDoneHomingFlag() {return doneHomingFlag;}
    double GetIndexFlag() {return indexFlag;}
    double GetMotorPos() {return motPos;}
    double GetMotorVel() {return motVel;}
    double GetPosError() {return posErr;}
    double GetTorpPos() {return torpPos;}
    double GetTorpVel() {return torpVel;}
    double GetDesVelCmd() {return desVel;}

    void SetRefPositionVelocity(double refPos, double refVel) // used in torpMasterControl
    {
        refPos = startPosAct + ((double) GetSignDir(flipSign)) * refPos;
        refVel = ((double) GetSignDir(flipSign)) * refVel;
    }

    int GetSignDir(bool flipSignFlag)
    {
        int signResult = (!flipSignFlag) ? 1 : -1;
        return signResult;
    }

    void SetReadySyncFlag(bool val) // used in torpMasterControl
    {
        readySync = val;
    }

    void SetSyncEnabledFlag(bool val) // used in torpMasterControl
    {
        syncEnabled = val;
    }

    void HaltMotion()
    {
        (*p_mm).HaltMotion();
    }

    int roundingFunc(double val) 
    {
        double threshold = 0.5;
        double diff = abs(val - (int) val);
        int result;
        if (val >= 0) // if positive
        {
            result = (int) (diff >= threshold) ? (int) val + 1 : (int) val;
        }
        else // if negative
        {
            result = (int) (diff >= threshold) ? (int) val - 1 : (int) val;
        }
    }

private:
    //configs
    TorpConfig config;
    
    //states
    static constexpr int INITIALIZING_TORP = 0;
    static constexpr int FINDING_HOME_INDEX = 1;
    static constexpr int MOVING_TO_STARTING_POS = 2;
    static constexpr int WAITING = 3;
    static constexpr int SYNCHRONIZING = 4;
    
    void InitializeLogs();
    std::ofstream auditTrailLog;
    std::ofstream torpControlLog;

    // torp maxon motors
    std::unique_ptr<MaxonMotor> p_mmL;
    std::unique_ptr<MaxonMotor> p_mmR;
    std::unique_ptr<MaxonMotor> p_mm;

    // encoder
    std::unique_ptr<LJEncoder3Channels> p_ljEnc3C;

    // PI Control
    std::unique_ptr<PIControl> p_pi;

protected:
    // torp ang pos, prev ang pos, ang vel, motor pos, motor vel, pos error
    double torpPos, torpPrePos, torpVel, motPos, motVel, posErr;

    double gearRatio = 1; // between motor and torp
    double time, t0, deltaT; // time value to calc integral
    double tA, tB;// time to change homing vel profile

    double preTime, timeStart, timeEnd; // synchronization variables

    // homing vel, max acc, acc mag, offset from home pos, offset pos lim, 
    double homingVel, maxAcc, accMag, offsetPos, offsetPosLim;
    //limit of proximity to start pos, home pos of torp, home pos of motor, ref start pos, actual start pos, 
    double startPosLim, homeTorpPos, homeMotPos, startPosRef, startPosAct;
    // ref pos, ref acc profile, ref vel profile, desired velocity to maxon
    double refPos, refAcc, refVel, desVel;

    bool flipSign; //CCW (+) OR CW (-)
    
    bool syncEnabled; // ready to sync flag

    // done homing, index flag, ready for sync flag, pos ready within start region flag, enabled to run in sync state
    bool doneHomingFlag, indexFlag, readySync, startPosRegionFlag, p_enableRunning;

    int movingAverageFilterSpan = 5;
    MovingAverage velMAFilter{movingAverageFilterSpan};
    //MovingAverage p_velMAFilter(p_movingAverageFilterSpan);

    unsigned int w = 25;


};

class TorpMasterControl : public BaseTask
{
public: 
    TorpMasterControl(bool startDeployBySoftwareFlag, 
                      std::unique_ptr<TorpControl> p_l_tc, 
                      std::unique_ptr<TorpControl> p_r_tc,
                      std::unique_ptr<StepperMotor> p_l_st1, 
                      std::unique_ptr<StepperMotor> p_l_st2, 
                      std::unique_ptr<StepperMotor> p_r_st1, 
                      std::unique_ptr<StepperMotor> p_r_st2);

    ~TorpMasterControl() override;
    int Run() override;

    double GetAccProfVal() {return accProfVal;}       // Function to get acceleration
    double GetVelProfVal() {return velProfVal;}       // Function to get velocity
    double GetJerkProfVal() {return jerkProfVal;}     // Function to get jerk value
    double GetPosProfVal() {return posProfVal;}        // Function to get position value
    bool GetDeployingDoneFlag() {return deployDoneFlag;}   // Function to get deploying done flag
    bool GetReTractingDoneFlag() {return retractDoneFlag;} // Function to get retracting done flag
    bool GetReadyToDeploySensorsFlag() {return readyToDeployFlag;}

    void SetDeployingFlag(bool flagVal)
    {
        deployStartFlag = flagVal;
    }

    void SetRetractingFlag(bool flagVal)
    {
        retractStartFlag = flagVal;
    }

    void ActuatingTorp(TorpControl *tc, double posVal, double velVal)
    {
        TorpControl *p_tc = NULL;
        p_tc = tc;

        (*p_tc).SetSyncEnabledFlag(true);

        // Set reference positon and velocity
        (*p_tc).SetRefPositionVelocity(posVal, velVal);

        // Actuate
        (*p_tc).Run();
    }


private:
    // config
    TorpConfig config;

    // states
    static constexpr int INITIALIZING = 0;
    static constexpr int HOMING = 1;
    static constexpr int ACCELERATING = 2;
    static constexpr int CRUISING = 3;
    static constexpr int DEPLOYING_MASS = 4;
    static constexpr int RETRACTING_MASS = 5;
    static constexpr int DECELERATING = 6;
    static constexpr int STOPPING = 7;


    void InitializeLogs();
    std::ofstream masterTorpLog;
    std::ofstream auditTrailLog;

    // pointers for torp control tasks
    std::unique_ptr<TorpControl> p_l_tc;
    std::unique_ptr<TorpControl> p_r_tc;
    
    // pointers for steppers
    std::unique_ptr<StepperMotor> p_l_st1; // left stepper 1
    std::unique_ptr<StepperMotor> p_l_st2; // left stepper 2
    std::unique_ptr<StepperMotor> p_r_st1; // right stepper 1
    std::unique_ptr<StepperMotor> p_r_st2; // right stepper 2
    
protected:
    
    // flags -- deployment turned on, start deploying, start retracting, homing finished,
    // deployment finished, retraction finished, ready to deploy
    bool deployBySoftwareFlag, deployStartFlag, retractStartFlag, doneHomingFlag, 
         deployDoneFlag, retractDoneFlag, readyToDeployFlag;
         
    bool softwareDeployFlag = true; // disable flag if no deployment desired

    // variables -- velocity magnitue [RPM], acc magnitude [RPM/s], time to acc [s], cruising time [s], 
    // time to deploy/retract [s], ending time for deploy/retract [s]
    double oprVelMag, maxAccMag = 1, tAccDec = 1, tCruise = 1, tDeployRetract = 65, tEndDeployRetract;

    // profile pos [deg], profile acc [RPM/s], profile vel [RPM], profile jerk [RPM/s^2]
    double posProfVal, accProfVal, velProfVal, jerkProfVal;
    
    // time values to calculate integral, 
    double time, preTime, deltaT, timeStart, timeEnd;

    // acc ramp-up, hold acc, acc ramp-down, cruising, interval times for profile [s]f
    double Ta, Tb, Tc, Td, T1, T2;

    unsigned int w = 25;

    TorpControl *p_l_tc = nullptr;
    TorpControl *p_r_tc = nullptr;

    StepperMotor *p_l_st1 = nullptr;
    StepperMotor *p_l_st2 = nullptr;
    StepperMotor *p_r_st1 = nullptr;
    StepperMotor *p_r_st2 = nullptr;

};