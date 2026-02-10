#include "tasks/TaskCoordinator.hpp"
#include "core/utils/TimeUtils.hpp"
#include "core/utils/Env.hpp"


using namespace std;
using namespace TimeUtils;

static double time_done = 0;
static double time_to_hold = envOrDefault("TIME_TO_HOLD", 10.0);

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
        cout << "[TaskCoordinator] Find sun finished, enabling torps" << endl;
    }
    // Check if torps are done spinning up. if it is, enable moving
    if (tc_.doneSpinningUp() && !ac_.movesEnabled())
    {   
        ac_.enableMove();
        cout << "[TaskCoordinator] Torps done spinning up, enabling moving" << endl;
    }
    // Once attitude control is done moving, we can spin down after holding for time hold
    if (ac_.movesDone() && !tc_.spinningDown())
    {   
        if (time_done == 0) {
            time_done = GetTimeNow();
            cout << "[TaskCoordinator] Moves done, starting hold timer" << endl;
        } else if (GetTimeNow() - time_done >= time_to_hold) {
            tc_.spinDown();
            cout << "[TaskCoordinator] Done moving, spinning down torps" << endl;
        }
    }

    nextTaskTime += deltaTaskTime;
    timeEnd = GetTimeNow();
    return 0;
}