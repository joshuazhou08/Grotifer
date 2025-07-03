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
    TorpControl(char* taskname,
                unsigned int taskID,
                homingProfilePara &hpp,
                PIControl &pi,
                MaxonMotor &mm,
                LJEncoder3Channels &ljEnc3C,
                bool useMotEncFlag,
                double gearRatio,
                ofstream &torpControlDataFile,
                ofstream &auditTrailDataFile);
    ~TorpControl() override;
    int Run() override;

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
    std::ofstream auditTrailDataFile;
    std::ofstream torpControlDataFile;

protected:
    // torp ang pos, prev ang pos, ang vel, motor pos, motor vel, pos error
    double p_torpPos, p_torpPrePos, p_torpVel, p_motPos, p_motVel, p_posErr;

    double p_gearRatio; // between motor and torp
    double p_time, p_preTime, p_deltaT; // time value to calc integral
    double p_tA, p_tB;// time to change homing vel profile

    // homing vel, max acc, acc mag, offset from home pos, offset pos lim, 
    double p_homingVel, p_maxAcc, p_accMag, p_offsetPos, p_offsetPosLim;
    //limit of proximity to start pos, home pos of torp, home pos of motor, ref start pos, actual start pos, 
    double p_startPosLim, p_homeTorpPos, p_homeMotPos, p_startPosRef, p_startPosAct;
    // ref pos, ref acc profile, ref vel profile, desired velocity to maxon
    double p_refPos, p_refAcc, p_refVel, p_desVel;

    bool p_flipSign; //CCW (+) OR CW (-)
    
    bool p_useMotEncFlag; // main enc is motor's encoder

    // done homing, index flag, ready for sync flag, pos ready within start region flag, enabled to run in sync state
    bool p_doneHomingFlag, p_indexFlag, p_readySync, p_startPosRegionFlag, p_enableRunning;

    int p_movingAverageFilterSpan;
    //MovingAverage p_velMAFilter(p_movingAverageFilterSpan);

    ofstream *p_torpControlDataFile;
    unsigned int w = 25;
};