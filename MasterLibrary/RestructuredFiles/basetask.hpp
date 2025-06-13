#ifndef BASETASK_HPP
#define BASETASK_HPP

#include <iostream>
#include <fstream>
#include <cstring>

// base class for all tasks
class BaseTask {
    public:
        // constructor -- name, ID
        BaseTask(char* taskName = nullptr, unsigned int taskID = 0);

        // destructor
        virtual ~BaseTask();

        // virtual run, implemented in derived tasks
        virtual int Run() = 0;

        // task info
        char* p_taskName;
        unsigned int p_taskID;

        // timing variables
        double p_deltaTaskTime;
        double p_nextTaskTime;

        // state machine variables
        unsigned int p_state;
        unsigned int p_nextState;

        // successful task flag
        bool p_runSuccess;

        // number of times task Run() called
        unsigned long p_numScans;

        // pointers to output files -- used by derived classes
        std::ofstream* p_auditTrailDataFile;

    protected:
        // possibly needed
  
};

// list element that holds a BaseTask pointer and links to next element
class TListElement {
    public:
        TListElement(BaseTask* taskPtr);
        ~TListElement();

        BaseTask* task;
        TListElement* next;

};

// linked list of tasks for scheduling
class TList {
    public:
        TList(const char* listName = nullptr);
        ~TList();

        // append task
        void Append(BaseTask* task);

        // iterate over list then return nullptr
        BaseTask* NextItem();

        // additional helpers
        bool IsEmpty() const;
        int Length() const;

    private:
        TListElement* first;
        TListElement* last;
        TListElement* current;
        int length;
        char* name;
};

#endif 

