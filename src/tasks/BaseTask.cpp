#include "BaseTask.hpp"
#include "Config.hpp"
#include "Devices.hpp"
#include <iostream>
#include <stdexcept>
#include <filesystem>

// -----------------------
// BaseTask implementation
// -----------------------

BaseTask::BaseTask(const char *name, int taskID)
    : name(name),
      taskID(taskID),
      deltaTaskTime(0.0),
      nextTaskTime(0.0),
      state(0),
      nextState(-1),
      numScans(0)
{
}

BaseTask::~BaseTask()
{
    std::cout << "Concluding " << name << " Task" << std::endl;
}

int BaseTask::Run() { return 0; }

void BaseTask::AuditTrailRecord() // Function to record the trail of the program
{
    unsigned int w = 25;
    duration = timeEnd - timeStart;
    * auditTrailLog << left << setw(w) << timeStart << left << setw(w) << timeEnd << left << setw(w) << duration*1e3 << left << setw(w) << taskName
    << left << setw(w) << taskID << left << setw(w) << stateName << left << setw(w) << nextStateName << endl;
}

void BaseTask::SetTimeToRunFlag(bool flagVal)
{
    timeToRunFlag = flagVal;
}
