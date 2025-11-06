#include "tasks/TaskCoordinator.hpp"

using namespace std;

TaskCoordinator::TaskCoordinator(AttitudeControl &ac,
                                 TorpControl &tc)
    : BaseTask("TaskCoordinator", 2),
      ac_(ac),
      tc_(tc)
{
    deltaTaskTime = 500e-3; // we can afford to have a massive deltaT cuz this isn't time sensitive
}

int TaskCoordinator::Run()
{
    if (GetTimeNow() < nextTaskTime)
    {
        return 0;
    }
    // update
    timeStart = GetTimeNow();
    state = nextState;
    stateName = nextStateName;

    // Check if find sun is done. if it is, enable the torps
    if (ac_.findSunDone() && !tc_.isEnabled())
    {
        tc_.enable();
    }
    // Check if torps are done spinning up. if it is, enable moving
    if (tc_.doneSpinningUp() && !ac_.movesEnabled())
    {
        ac_.enableMove();
    }
    // Once attitude control is done moving, we can spin down.
    if (ac_.movesDone())
    {
        tc_.spinDown();
    }

    nextTaskTime += deltaTaskTime;
    timeEnd = GetTimeNow();
    return 0;
}