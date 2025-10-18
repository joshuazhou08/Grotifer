#include "core/BaseTask.hpp"
#include <iomanip>

// -----------------------
// BaseTask implementation
// -----------------------
using namespace std;
BaseTask::BaseTask(const char *name, int taskID)
    : name(name),
      taskID(taskID),
      deltaTaskTime(0.0),
      nextTaskTime(0.0),
      state(0),
      nextState(0),
      numScans(0) {};

BaseTask::~BaseTask()
{
    std::cout << "Concluding " << name << " Task" << std::endl;
}

int BaseTask::Run() { return 0; }