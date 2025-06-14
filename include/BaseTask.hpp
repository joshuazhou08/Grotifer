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

    // Task timing
    double deltaTaskTime; // Task period
    double nextTaskTime;  // Time for next execution

    // State variables for simple state machines
    int state;
    int nextState;
    int numScans;

    // log file line width
    unsigned int w = 25;

public:
    BaseTask(const char *name, int taskID = 0);
    virtual ~BaseTask();

    virtual int Run(void);

    int GetTaskID();
};
