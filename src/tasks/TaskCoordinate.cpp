#include "Tasks.hpp"
#include "Config.hpp"
#include "Devices.hpp"
#include <iostream>
#include <stdexcept>
#include <filesystem>

TaskCoordinate::TaskCoordinate(std::unique_ptr<TorpControl> p_l_tc,
                               std::unique_ptr<TorpControl> p_r_tc,
                               std::unique_ptr<TorpMasterControl> p_tmc,
                               std::unique_ptr<AttitudeControl> p_attc,
                               ofstream auditTrailLog)
    : BaseTask("TaskCoordinate", 3)
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

    auditTrailLog.open("logs/auditTrailLog.txt", std::ios::out | std::ios::app);

    auditTrailLog << left << setw(w) << "Start" << left << setw(w) << "End" << left << setw(w) << "Duration[ms]" << left << setw(w) << "TaskName"
                  << left << setw(w) << "TaskID" << left << setw(w) << "CurrState" << left << setw(w) << "NextState" << endl;
}

int TaskCoordinate::Run()
{
    if (GetTimeNow() < nextTaskTime)
    {
        return 0;
    }
    
    timeStart = GetTimeNow();
    nextTaskTime = deltaTaskTime;

    switch (state)
    {
        case INITIALIZING:
            if (sensorsOnlyFlag)
            {
                nextState = DEPLOYING;
            }
            else
            {
                if (!controlBodyNoFindSunFlag)
                {
                    if ((*p_attc).GetFindingSunDoneFlag()) // if there is a flag for this???
                    {
                        if (!sensorsOnlyFlag)
                        {
                            // other flag functions in attitude control class
                            
                        }
                    }
                }
            }
    }

}