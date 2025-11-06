// Header files for all the tasks
#pragma once
#include "core/BaseTask.hpp" 
#include "tasks/AttitudeControlTask.hpp"
#include "tasks/TorpControlTask.hpp"

// Task to set flags based on other task flags to "coordinate" the tasks
class TaskCoordinator : public BaseTask
{
public:
    TaskCoordinator (AttitudeControl& ac, TorpControl& tc);
    
    int Run() override;

private: 
    // store by reference, the lifespan of these objects are handled by main
    AttitudeControl& ac_;
    TorpControl& tc_;
}