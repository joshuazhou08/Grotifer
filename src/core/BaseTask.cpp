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
      state(INITIALIZING),
      nextState(INITIALIZING),
      numScans(0) {};

BaseTask::~BaseTask()
{
    std::cout << "Concluding " << name << " Task" << std::endl;
    stopLogger();
}

int BaseTask::Run() { return 0; }

void BaseTask::startLogger() {
    running_ = true;
    writer_ = std::thread(&BaseTask::loggerLoop, this);
}

void BaseTask::stopLogger() {
    running_ = false;
    if (writer_.joinable())
        writer_.join();
}

void BaseTask::loggerLoop() {
    using namespace std::chrono_literals;
    while (running_) {
        for (auto& q : queues_)
            q->flush();
        std::this_thread::sleep_for(10ms);
    }
    for (auto& q : queues_)
        q->flush(); // final drain
}

