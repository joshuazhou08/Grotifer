#include "Tasks.hpp"
#include "Config.hpp"
#include "Devices.hpp"
#include <iostream>
#include <stdexcept>
#include <filesystem>

TorpControl::TorpControl(homingProfilePara homingPara, 
                        std::unique_ptr<PIControl> p_pi, 
                        std::unique_ptr<MaxonMotor> p_mm, 
                        std::unique_ptr<LJEncoder3Channels> p_ljEnc3C,
                        double gearRatio)
    : BaseTask("TorpControl", 1)
                                
{
    InitializeLogs();
    if (!torpControlLog)
    {
        throw std::runtime_error("Failed to open one or more log files in TorpControl.");
    }

    deltaTaskTime = config.deltaTaskTime;
    homingVel = homingPara.homingVel;
    maxAcc = homingPara.maxAcc;
    offsetPos = homingPara.offsetPos;
    offsetPosLim = homingPara.offsetPosLim;
    startPosLim = homingPara.startPosLim;
    flipSign = (homingVel >= 0) ? false : true;
    nextTaskTime = 0;
    state = INITIALIZING_TORP;
    nextState = INITIALIZING_TORP;

}

TorpControl::~TorpControl()
{
    if (torpControlLog.is_open()) torpControlLog.close();
}

void TorpControl::InitializeLogs()
{
    std:filesystem::create_directories("logs");

    torpControlLog.open("logs/torpcontrol.txt", std::ios::out | std::ios::app);

    torpControlLog << left << setw(w) << "Time (s)" << left << setw(w) << "Home Torp Position (deg)" << left << setw(w) << "Staring Position Ref (deg)"
                                                      << left << setw(w) << "Reference Position (deg)" << left << setw(w) << "Torp Position (deg)"
                                                      << left << setw(w) <<"Reference Velocity (RPM)" << left << setw(w) << "Torp Velocity (RPM)"
                                                      << left << setw(w) << "Motor Position (deg)" << left << setw(w) << "Motor Velocity (RPM)"
                                                      << left << setw(w) << "Position Error (deg)" << left << setw(w) << "Commanded Velocity (RPM)" << endl;
}

int TorpControl::Run()
{
    if (GetTimeNow() < nextTaskTime)
    {
        return 0;
    }

    time = GetTimeNow();

    if (state == SYNCHRONIZING) goto syncExecution;

    switch (state)
    {
        case INITIALIZING_TORP: // assuming automatic homing via software

            nextState = FINDING_HOME_INDEX;
            tA = GetTimeNow();
            tB = tA + abs(homingVel / maxAcc) - deltaTaskTime;
            refAcc = ((double) GetSignDir(flipSign)) * maxAcc;
            t0 = GetTimeNow();

            break;

        case FINDING_HOME_INDEX: // entry code
            
            motPos = (*p_mm).GetPositionIs();
            motVel = (*p_mm).GetVelocityIs();
            torpPos = (*p_ljEnc3C).GetAngularPosDeg();
            torpVel = velMAFilter.addSample(((torpPos - torpPrePos)/ 360.0) / (deltaT / 60.0));
            torpPrePos = torpPos;

            if (GetTimeNow() > tB)
            {
                refAcc = 0;
                refVel = homingVel;
            } 
            else 
            {
                refVel = refVel + refAcc * deltaT;
            }

            if ((*p_ljEnc3C).GetIndexFlag()) 
            {
                indexFlag = true;
                nextState = MOVING_TO_STARTING_POS;
                homeTorpPos = torpPos;
                homeMotPos = motPos;

                // removed use motor encoder flag and logic since it is not used
                startPosRef = (homeTorpPos + ((double) GetSignDir(flipSign)) * offsetPos);
                
                if (abs(offsetPos) >= offsetPosLim)
                {
                    double a_min = abs((pow(refVel, 2) / (2*offsetPos)) * 6);
                    if (a_min >= maxAcc) accMag = a_min;
                    else if (a_min <= 0.2 * maxAcc) accMag = maxAcc;
                    else accMag = 5.0 * a_min;

                    tA = abs((offsetPos / CONV_RAD_TO_DEG) / (refVel * CONV_RPM_TO_RADpSEC)) + GetTimeNow() - abs(refVel / (2 * accMag));
                    tB = abs((offsetPos / CONV_RAD_TO_DEG) / (refVel * CONV_RPM_TO_RADpSEC)) + GetTimeNow() + abs(refVel / (2 * accMag));
                }
                else
                {
                    accMag = 0;
                    tA = GetTimeNow(); 
                    tB = tA;
                }
            
            }

            break;

        case MOVING_TO_STARTING_POS:

            motPos = (*p_mm).GetPositionIs();
            motVel = (*p_mm).GetVelocityIs();
            torpPos = (*p_ljEnc3C).GetAngularPosDeg();
            torpVel = velMAFilter.addSample(((torpPos - torpPrePos) / 360.0) / (deltaT / 60.0));
            torpPrePos = torpPos;

            refAcc = ((GetTimeNow() >= tA && GetTimeNow() <= tB) ? (((double) GetSignDir(flipSign)) * (-accMag)) : 0);
            refVel = ((GetTimeNow() > tB) ? 0 : (refVel + refAcc * deltaT)); 

            // removed the motor encoder flag and logic, never used
            if (abs(torpPos - startPosRef) <= 1.25 * startPosLim || abs(refVel) <= 0.035)
            {
                if ((abs(torpPos - startPosRef) <= startPosLim) && (abs(refVel) <= 0.01) && (abs(torpVel) <= 0.25))
                {
                    nextState = WAITING;
                    startPosAct = startPosRef;
                    (*p_mm).HaltMotion();
                }
                startPosRegionFlag = true;
            }

            break;

        case WAITING:
            motPos = (*p_mm).GetPositionIs();
            motVel = (*p_mm).GetVelocityIs();
            torpPos = (*p_ljEnc3C).GetAngularPosDeg();
            torpVel = velMAFilter.addSample(((torpPos - torpPrePos) / 360.0)/ (deltaT / 60.0));
            torpPrePos = torpPos;

            doneHomingFlag = true;

            refAcc = 0;
            refVel = 0;

            if (readySync)
            {
                state = SYNCHRONIZING;
                nextState = SYNCHRONIZING;
            }

            break;

        case SYNCHRONIZING:
            syncExecution:
                if (syncEnabled)
                {

                    // time data for runtime and integration
                    timeStart = GetTimeNow();
                    time = GetTimeNow();
                    deltaT = time - preTime;
                    
                    // pull torp data
                    motPos = (*p_mm).GetPositionIs();
                    motVel = (*p_mm).GetVelocityIs();
                    torpPos = (*p_ljEnc3C).GetAngularPosDeg();
                    torpVel = velMAFilter.addSample(((torpPos - torpPrePos) / 360.0)/ (deltaT / 60.0));
                    torpPrePos = torpPos;
                    
                    // PI Control for velocity
                    desVel = (*p_pi).PICalculation(refPos, torpPos) / CONV_TURN2DEG + refVel;
                    posErr = (*p_pi).GetError();

                    // motor actuation
                    (*p_mm).RunWithVelocity(roundingFunc(desVel * gearRatio));

                    // flip flag
                    syncEnabled = false;

                    // log data
                    torpControlLog << left << setw(w) << GetTimeNow() << left << setw(w) << GetHomeTorpPos() << left << setw(w) << GetStartingPosRef()
                                                                      << left << setw(w) << GetRefPosition() << left << setw(w) << GetTorpPos()
                                                                      << left << setw(w) << GetRefVelocity() << left << setw(w) << GetTorpVel()
                                                                      << left << setw(w) << GetMotorPos() << left << setw(w) << GetMotorVel()
                                                                      << left << setw(w) << GetPosError() << left << setw(w) << GetDesVelCmd() << endl;


                    timeEnd = GetTimeNow();
                    AuditTrailRecord();
                    return 1;
                } 
                else 
                {
                    cout << "Torp Synchronization not currently allowed by syncEnabled flag." << endl;
                }
                break;
                
            
        preTime = time;
    }

    // calculate reference position
    refPos = (startPosRegionFlag || doneHomingFlag) ? startPosRef : (refPos + refVel * (deltaT / 60.0) * (CONV_TURN2DEG));

    // Run PI control
    desVel = (*p_pi).PICalculation(refPos, torpPos) / CONV_TURN2DEG + refVel;
    posErr = (*p_pi).GetError();

    // actuate motor
    (*p_mm).RunWithVelocity(roundingFunc(desVel * gearRatio));
    
    // log data
    torpControlLog << left << setw(w) << GetTimeNow() << left << setw(w) << GetHomeTorpPos() << left << setw(w) << GetStartingPosRef()
                                                                      << left << setw(w) << GetRefPosition() << left << setw(w) << GetTorpPos()
                                                                      << left << setw(w) << GetRefVelocity() << left << setw(w) << GetTorpVel()
                                                                      << left << setw(w) << GetMotorPos() << left << setw(w) << GetMotorVel()
                                                                      << left << setw(w) << GetPosError() << left << setw(w) << GetDesVelCmd() << endl;

    timeEnd = GetTimeNow();
    AuditTrailRecord();
    
                                                                      // serial output for current state
    state = nextState;
    std::string nextStateString;
    if (nextState != state)
    {
        switch (nextState)
        {
            case INITIALIZING_TORP:
            {
                nextStateString = "Homing Sequence -- Initializing Torp";
            }
            case FINDING_HOME_INDEX:
            {
                nextStateString = "Homing Sequence -- Finding Home Index";
            }
            case MOVING_TO_STARTING_POS:
            {
                nextStateString = "Homing Sequence -- Moving to Starting Position";
            }
            case WAITING:
            {
                nextStateString = "Homing Sequence -- Waiting";
            }
            case SYNCHRONIZING:
            {
                nextStateString = "Homing Sequence -- Synchronizing";
            }
        }
        std::cout << "[Torp Task] Switching to state: " << nextStateString << std::endl;
    }

    nextTaskTime += deltaTaskTime;
    return 0;

}




// *********** FLESH OUT FUNCTIONS USED ******************