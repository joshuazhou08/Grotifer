#pragma once
#include <iostream>
#include <memory>

// -----------------------
// BaseTask class
// -----------------------
class BaseTask
{
protected:
    const char *name;
    int taskID;
    char* taskName;
    // Task timing
    double deltaTaskTime; // Task period
    double nextTaskTime;  // Time for next execution

    // **************** IS THIS NEEDED? ****************
    double currTime; // Current time of task

    // State variables for simple state machines
    int state;
    int nextState;
    int numScans;

    // log file variables
    unsigned int w = 25;
    char* stateName;
    char* nextStateName;

    std::ofstream *auditTrailLog = NULL;

public:
    BaseTask(const char *name, int taskID = 0);
    virtual ~BaseTask();

    virtual int Run(void);

    double duration, timeEnd, timeStart;

    void AuditTrailRecord();

};

// task ID: 0 = attitude control, 1 = torp control, 2 = torp master control
// ************* DEFINE GetTaskID and AuditTrailRecord fucntions ***************