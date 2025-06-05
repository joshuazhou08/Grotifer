#include "basetask.hpp"
#include "tasks.hpp"
#include "timerutilities.hpp"

CTask1::CTask1(char *aName) : BaseTask(aName) {
    DeltaTaskTime = 0.1;  // Task period
    State = 0;
    NextState = 0;
    // Initialize variables here
}

CTask1::~CTask1() {}

int CTask1::Run(void) {
    double Time;
    NumScans++;

    if (State == 0) {
        // Initialization
        State = 1;
        NextState = 1;
        return 0;
    }

    if ((Time = GetTimeNow()) < NextTaskTime) return 1; // Wait until next scheduled time
    NextTaskTime += DeltaTaskTime;

    if (NextState != -1) {
        State = NextState;
        NextState = -1;
        RunEntry = 1;
    } else {
        RunEntry = 0;
    }

    switch (State) {
        case 1:
            if (RunEntry) {
                // Entry code here
            }
            // Action code here
            // Transition logic
            
            if (!RunEntry) { // condition added here
                NextState = 2;
            }
            break;
        case 2:
            // ...
            break;
        default:
            // Illegal state error
            return -1;
    }
    return 0;
}