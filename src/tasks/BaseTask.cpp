#include "BaseTask.hpp"
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
      numScans(0)
{
    std::filesystem::create_directories("logs");
    auditTrailLog.open("logs/trail_record.txt", std::ios::out | std::ios::trunc);
}

BaseTask::~BaseTask()
{
    std::cout << "Concluding " << name << " Task" << std::endl;
}

void BaseTask::AuditDataTrail()
{
    double duration = timeEnd - timeStart;
    auditTrailLog << left << setw(w) << timeStart << left << setw(w) << timeEnd << left << setw(w) << duration * 1e3 << left << setw(w) << name
                  << left << setw(w) << taskID << left << setw(w) << stateName << left << setw(w) << nextStateName << endl;
}

int BaseTask::Run() { return 0; }