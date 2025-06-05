#ifndef BASETASK_HPP
#define BASETASK_HPP

#include <iostream>
#include <fstream>
#include <cstring>

// Forward declarations
class BaseTask;

// -----------------------
// BaseTask class
// -----------------------
class BaseTask {
public:
    BaseTask(char* aName, int taskID = 0);
    virtual ~BaseTask();

    virtual int Run(void);      // Virtual: must be overridden by derived task classes

    // For task management
    char* Name;
    int TaskID;

    // Task timing
    double DeltaTaskTime;       // Task period
    double NextTaskTime;        // Time for next execution

    // State variables for simple state machines
    int State;
    int NextState;
    int RunEntry;

    // Execution stats
    long NumScans;

    // Functions for profiling, messaging, etc. could be added here

protected:
    // Protected members to support derived classes

private:
    // Private members to protect internal data
};

// -----------------------
// Linked list element for task list
// -----------------------
class TListElement {
public:
    TListElement(BaseTask* ItemPointer);
    ~TListElement();

    BaseTask* Item;
    TListElement* Next;
};

// -----------------------
// Linked list to hold tasks
// -----------------------
class TList {
public:
    TList(const char* listName = nullptr);
    ~TList();

    void Append(BaseTask* Item);
    BaseTask* NextItem();
    int IsEmpty() const;
    int HowLong() const;

private:
    TListElement* First;
    TListElement* Last;
    TListElement* Current;
    int Length;
    char* Name;
};

#endif  // BASETASK_HPP
