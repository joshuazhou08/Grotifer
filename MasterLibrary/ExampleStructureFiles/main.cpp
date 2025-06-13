#include <iostream>
#include "tasks.hpp"      // Task class declarations
#include "basetask.hpp"   // BaseTask and task list classes
#include "timerutilities.hpp" // For GetTimeNow()

int main() {
    // Create task lists by priority (example: low and intermittent)
    TList* LowPriority = new TList("Low Priority");
    TList* Intermittent = new TList("Intermittent");

    // Instantiate the tasks
    Task1 = new CTask1("Task1");
    Task2 = new CTask2("Task2");
    Task3 = new CTask3("Task3");

    // Initialize task timing and states (could also be in constructors)
    Task1->DeltaTaskTime = 0.1;  // Run every 0.1 sec
    Task2->DeltaTaskTime = 0.2;  // Run every 0.2 sec
    Task3->DeltaTaskTime = 0.5;  // Run every 0.5 sec

    Task1->NextTaskTime = 0.0;
    Task2->NextTaskTime = 0.0;
    Task3->NextTaskTime = 0.0;

    // Add tasks to priority lists
    LowPriority->Append(Task1);
    LowPriority->Append(Task2);
    Intermittent->Append(Task3);

    // Example scheduler main loop
    double EndTime = 5.0;  // Run for 5 seconds
    while (GetTimeNow() < EndTime) {
        // Run all tasks in LowPriority list
        BaseTask* task;
        while ((task = LowPriority->NextItem()) != nullptr) {
            int result = task->Run();
            if (result < 0) {
                std::cerr << "Error running task: " << (task->Name ? task->Name : "Unknown") << "\n";
                break;
            }
        }
        // Run all tasks in Intermittent list
        while ((task = Intermittent->NextItem()) != nullptr) {
            int result = task->Run();
            if (result < 0) {
                std::cerr << "Error running task: " << (task->Name ? task->Name : "Unknown") << "\n";
                break;
            }
        }
    }

    // Cleanup tasks and lists
    delete Task1;
    delete Task2;
    delete Task3;

    delete LowPriority;
    delete Intermittent;

    std::cout << "Program completed.\n";
    return 0;
}