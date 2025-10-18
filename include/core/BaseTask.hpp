#pragma once
#include <iostream>
#include <memory>
#include <filesystem>
#include <fstream>
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

    double timeStart;
    double timeEnd;

    // State variables for simple state machines
    int state;
    std::string stateName;
    int nextState;
    std::string nextStateName;
    int numScans;
    
public:
    BaseTask(const char *name, int taskID = 0);
    virtual ~BaseTask();

    virtual int Run(void);

    int GetTaskID();
};
