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

public:
    BaseTask(const char *name, int taskID = 0);
    virtual ~BaseTask();

    virtual int Run(void);

    int GetTaskID();
};

/* Task circular linked list for easy scheduling */
struct TaskNode
{
    std::unique_ptr<BaseTask> task;
    TaskNode *next;

    TaskNode(std::unique_ptr<BaseTask> t);
};

// Circular linked list
class TaskList
{
private:
    TaskNode *tail = nullptr;

public:
    ~TaskList();

    void insert(std::unique_ptr<BaseTask> t);
};