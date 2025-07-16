#include "Tasks.hpp"
#include "Config.hpp"
#include "Devices.hpp"
#include <iostream>
#include <stdexcept>
#include <filesystem>

TorpMasterControl::TorpMasterControl(bool startDeployBySoftwareFlag, 
                                     std::unique_ptr<TorpControl> p_l_tc, 
                                     std::unique_ptr<TorpControl> p_r_tc,
                                     std::unique_ptr<StepperMotor> p_l_st1, 
                                     std::unique_ptr<StepperMotor> p_l_st2, 
                                     std::unique_ptr<StepperMotor> p_r_st1, 
                                     std::unique_ptr<StepperMotor> p_r_st2)
    : BaseTask("TorpMasterControl", 2)
{
    InitializeLogs();
    if (!masterTorpLog)
    {
        throw std::runtime_error("Failed to open one or more log files in MasterTorpControl.");
    }

    deltaTaskTime = config.deltaTaskTime;
    nextTaskTime = 0;
    oprVelMag = config.oprVel;
    tAccDec = config.tAccDec;
    tCruise = config.tCruise;
    softwareDeployFlag = startDeployBySoftwareFlag; // deployment turned on/off for this run
    double maxAccAllowed = abs(2 * oprVelMag / tAccDec);
    if (config.maxAccMag < maxAccAllowed)
    {
        maxAccMag = config.maxAccMag;
    }
    else
    {
        cout << "Max accerlation magnitude exceeds allowable value, has been set to max allowed acceleration." << endl;
        maxAccMag = maxAccAllowed;
    }
    state = INITIALIZING;
    nextState = state;
}

TorpMasterControl::~TorpMasterControl()
{
    if (masterTorpLog.is_open())
        masterTorpLog.close();
    if (auditTrailLog.is_open())
        auditTrailLog.close();
}

void TorpMasterControl::InitializeLogs()
{
    std::filesystem::create_directories("logs");

    masterTorpLog.open("logs/masterTorpLog.txt", std::ios::out | std::ios::app);

    masterTorpLog << left << setw(w) << "Time (s)" << left << setw(w) << "Position Profile Value (deg)" 
                  << left << setw(w) << "Velocity Profile Value (RPM)" << left << setw(w) << "Acceleration Profile Value (RPM/s)" << endl;
}

int TorpMasterControl::Run()
{
    if (GetTimeNow() < nextTaskTime)
    {
        return 0;
    }

    preTime = GetTimeNow();
    timeStart = GetTimeNow();
    nextTaskTime += deltaTaskTime;

    time = GetTimeNow();
    deltaT = time - preTime;
    preTime = time;
    state = nextState;

    switch (state)
    {
        case INITIALIZING:
            nextState = HOMING;
            break;

        case HOMING:
            if ((*p_l_tc).GetDoneHomingFlag() && (*p_r_tc).GetDoneHomingFlag())
            {
                doneHomingFlag = true;

                // sync flag set
                (*p_l_tc).SetReadySyncFlag(true);
                (*p_r_tc).SetReadySyncFlag(true);

                nextState = ACCELERATING;

                T2 = abs(2 * oprVelMag / maxAccMag) - tAccDec;
                T1 = 0.5 * (tAccDec - T2);

                Ta = time + T1;
                Tb = time + T1 + T2;
                Tc = time + tAccDec;

                jerkProfVal = abs(maxAccMag / T1);

            }
            break;

        case ACCELERATING:
            
            if (time >= Tc)
            {
                nextState = CRUISING;
                readyToDeployFlag = true;

                Td = time + tCruise;
                velProfVal = oprVelMag;
                accProfVal = 0;
                jerkProfVal = 0;

            }
            else
            {
                if (time >= Ta && time <= Tb)
                {
                    jerkProfVal = 0;
                }
                else if (time > Tb)
                {
                    jerkProfVal = -abs(maxAccMag / T1);
                }
            }

            break;
        
        case CRUISING:
            
            if (time >= Td)
            {
                if (deployBySoftwareFlag)
                {
                    if (deployStartFlag && !deployDoneFlag)
                    {
                        nextState = DEPLOYING_MASS;
                        tEndDeployRetract = time + tDeployRetract;
                        velProfVal = oprVelMag;
                        accProfVal = 0;
                        jerkProfVal = 0;
                    }
                    else if (retractStartFlag && !retractDoneFlag)
                    {
                        nextState = RETRACTING_MASS;
                        tEndDeployRetract = time + tDeployRetract;
                        velProfVal = oprVelMag;
                        accProfVal = 0;
                        jerkProfVal = 0;
                    }
                    else if (retractDoneFlag && deployDoneFlag)
                    {
                        nextState = DECELERATING;
                        Ta = time + T1;
                        Tb = time + T1 + T2;
                        Tc = time + tAccDec;
                        jerkProfVal = -abs(maxAccMag / T1);
                    }
                    else
                    {
                        velProfVal = oprVelMag;
                        accProfVal = 0;
                        jerkProfVal = 0;

                    }
                }
                else
                {
                    nextState = DECELERATING;
                    Ta = time + T1;
                    Tb = time + T1 + T2;
                    Tc = time + tAccDec;
                    jerkProfVal = -abs(maxAccMag / T1);
                }
            }

            break;

        case DEPLOYING_MASS:
            
            if (!deployDoneFlag) // time to deploy
            {
                double distPerStep = 0.04; // linear distance per step (mm)
                double lBoom = 200; // travel distance of each boom
                int32_t deployTargetPos = (int32_t) lBoom / distPerStep;

                (*p_l_st1).RunToTargetPosition(deployTargetPos);
                (*p_l_st2).RunToTargetPosition(deployTargetPos);
                (*p_r_st1).RunToTargetPosition(deployTargetPos);
                (*p_r_st2).RunToTargetPosition(deployTargetPos);

            }

            velProfVal = oprVelMag;
            accProfVal = 0;
            jerkProfVal = 0;

            if (time >= tEndDeployRetract)
            {
                nextState = CRUISING;
                Td = time + tCruise;
                deployDoneFlag = true;
            }

            break;

        case RETRACTING_MASS:
            
            if (!retractDoneFlag)
            {
                double distPerStep = 0.04;
                double lBoom = 200;
                double lOffset = 25;
                int32_t retractTargetPos = -(int32_t) (lBoom - 10) / distPerStep;

                (*p_l_st1).RunToTargetPosition(retractTargetPos);
                (*p_l_st2).RunToTargetPosition(retractTargetPos);
                (*p_r_st1).RunToTargetPosition(retractTargetPos);
                (*p_r_st2).RunToTargetPosition(retractTargetPos);

                retractDoneFlag = true;
            }

            velProfVal = oprVelMag;
            accProfVal = 0;
            jerkProfVal = 0;

            if (time >= tEndDeployRetract)
            {
                (*p_l_st1).DeEnergize();
                (*p_l_st2).DeEnergize();
                (*p_r_st1).DeEnergize();
                (*p_r_st2).DeEnergize();

                nextState = CRUISING;
                Td = time + tCruise;
            }

            break;

        case DECELERATING:

            if (time >= Tc)
            {
                nextState = STOPPING;
                velProfVal = 0;
                accProfVal = 0;
                jerkProfVal = 0;
            }
            else
            {
                if (time >= Ta && time <= Tb)
                {
                    jerkProfVal = 0;
                }
                else if (time > Tb)
                {
                    jerkProfVal = abs(maxAccMag / T1);
                }
            }

            break;

        case STOPPING:

            (*p_l_tc).HaltMotion();
            (*p_r_tc).HaltMotion();

            break;

    }

    if (doneHomingFlag)
    {
        accProfVal = accProfVal + jerkProfVal * deltaT;
        velProfVal = velProfVal + accProfVal * deltaT;
        posProfVal = posProfVal + (velProfVal * (deltaT / 60.0) * 360.0);

        ActuatingTorp(p_l_tc, posProfVal, velProfVal);
        ActuatingTorp(p_r_tc, posProfVal, velProfVal);
        
    }

    masterTorpLog << left << setw(w) << GetTimeNow() << left << setw(w) << GetPosProfVal() << left 
                  << setw(w) << GetVelProfVal() << left << setw(w) << GetAccProfVal() << endl;

    timeEnd = GetTimeNow();
    AuditTrailRecord();
    return 1;

}

