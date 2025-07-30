#include "Tasks.hpp"
#include "Config.hpp"
#include "Devices.hpp"
#include <iostream>
#include <stdexcept>
#include <filesystem>
#include <ios>
#include <iomanip>

TaskCoordinate::TaskCoordinate(std::shared_ptr<TorpControl> l_tc,
                               std::shared_ptr<TorpControl> r_tc,
                               std::shared_ptr<TorpMasterControl> tmc,
                               std::shared_ptr<AttitudeControl> attc)
    : BaseTask("TaskCoordinate", 3),
    p_l_tc(std::move(l_tc)),
    p_r_tc(std::move(r_tc)),
    p_tmc(std::move(tmc)),
    p_attc(std::move(attc))

{   
    InitializeLogs();

    if (!auditTrailLog)
    {
        throw std::runtime_error("Failed to open one or more log files in Task Control.");
    }

    deltaTaskTime = config.deltaTaskTime;
    holdPosAfterDeployFlag = config.holdPosAfterDeployFlag;
    sensorsOnlyFlag = config.sensorsOnlyFlag;
    controlBodyOnlyFlag = config.controlBodyOnlyFlag;
    controlBodyNoFindSunFlag = config.controlBodyNoFindSunFlag;

    state = INITIALIZING;
    nextState = INITIALIZING;
}

TaskCoordinate::~TaskCoordinate()
{
    if (auditTrailLog.is_open())
        auditTrailLog.close();
}

void TaskCoordinate::InitializeLogs()
{
    std::filesystem::create_directories("logs");
    auditTrailLog.open("logs/masterTorpLog.txt", std::ios::out | std::ios::app);

    auditTrailLog << left << setw(w) << "Start" << left << setw(w) << "End" << left << setw(w) << "Duration[ms]" << left << setw(w) << "TaskName"
                  << left << setw(w) << "TaskID" << left << setw(w) << "CurrState" << left << setw(w) << "NextState" << endl;
}

int TaskCoordinate::Run()
{
    if (!timeToRunFlag)
    {
        return -1;
    }

    if (GetTimeNow() < nextTaskTime)
    {
        return 1;
    }
    
    timeStart = GetTimeNow();
    state = nextState;

    switch (state)
    {
        case INITIALIZING:
            if (sensorsOnlyFlag)
            {
                nextState = DEPLOYING;
            }
            else
            {
                if ((*p_attc).GetFindingSunDoneFlag()) // ****we need a flag and function for this
                {
                    if (!controlBodyOnlyFlag)
                    {
                        (*p_tmc).SetTimeToRunFlag(true);
                        if (!(*p_tmc).GetFirstTimeRunFlag())
                        {
                            nextState = DEPLOYING;
                            (*p_l_tc).SetTimeToRunFlag(true);
                            (*p_r_tc).SetTimeToRunFlag(true);
                        }
                            
                    }
                    else
                    {
                        nextState = MOVING;
                        (*p_l_tc).SetTimeToRunFlag(false);
                        (*p_r_tc).SetTimeToRunFlag(false);
                        (*p_tmc).SetTimeToRunFlag(false);
                    }
                }

                else
                {
                    nextState = INITIALIZING;
                    (*p_l_tc).SetTimeToRunFlag(false);
                    (*p_r_tc).SetTimeToRunFlag(false);
                    (*p_tmc).SetTimeToRunFlag(false);
                }
            }

            break;
            
        case DEPLOYING:

            if (!holdPosAfterDeployFlag)
            {
                if ((*p_tmc).GetReadyToDeploySensorsFlag())
                {
                    if (tStartDeploySensors <= 0)
                    {
                        tStartDeploySensors = GetTimeNow() + 10.0;
                    }
                    else
                    {
                        if (GetTimeNow() >= tStartDeploySensors)
                        {
                            (*p_tmc).SetDeployingFlag(true);
                        }
                        else
                        {
                            (*p_tmc).SetDeployingFlag(false);
                        }
                    }
                }

                if ((*p_tmc).GetDeployingDoneFlag())
                {
                    nextState = MOVING;
                    (*p_attc).SetStartMovingFlag(true); // ** add this flag to attitude control
                }

                else (*p_attc).SetStartMovingFlag(false);
            }

            else (*p_attc).SetStartMovingFlag(false);

            break;

        case MOVING:
            
            if (controlBodyOnlyFlag)
            {
                (*p_l_tc).SetTimeToRunFlag(false);
                (*p_r_tc).SetTimeToRunFlag(false);
                (*p_tmc).SetTimeToRunFlag(false);
            }

            (*p_attc).SetStartMovingFlag(true);

            if ((*p_attc).GetDoneMovingFlag()) // ** add this function/flag to attitude control
            {
                nextState = STOPPING;

                (*p_tmc).SetRetractingFlag(true);
            }
            
            break;

        case STOPPING:
            
            if (controlBodyOnlyFlag)
            {
                (*p_l_tc).SetTimeToRunFlag(false);
                (*p_r_tc).SetTimeToRunFlag(false);
                (*p_tmc).SetTimeToRunFlag(false);
            }

            (*p_attc).SetStartMovingFlag(true);

            break;
    }

    state = nextState;
    std::string nextStateString;
    if (nextState != state)
    {
        switch (nextState)
        {
        case INITIALIZING:
        {
            nextStateString = "Deploying";
        }
        case DEPLOYING:
        {
            nextStateString = "Moving";
        }
        case MOVING:
        {
            nextStateString = "Stopping";
        }
        case STOPPING:
        {
            nextStateString = "Stopping";
        }
        }
        std::cout << "[Task Coordinate Task] Switching to state: " << nextStateString << std::endl;
        
    }
    nextTaskTime += deltaTaskTime;
    timeEnd = GetTimeNow();
    AuditTrailRecord();
    return 0;

}