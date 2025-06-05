// Example tasks.hpp header for program with 3 tasks

#ifndef TASKS_HPP
#define TASKS_HPP

#include "basetask.hpp"  // BaseTask class declaration

// Task 1
class CTask1 : public BaseTask {
public:
    CTask1(char *aName);    // Constructor
    ~CTask1();              // Destructor

    int Run(void);          // Main task function called by scheduler

private:
    // Task1-specific variables here
    double xValue;          // Example exchange variable
};

// Task 2
class CTask2 : public BaseTask {
public:
    CTask2(char *aName);
    ~CTask2();

    int Run(void);

private:
    // Task2-specific variables here
    double xData;
};

// Task 3
class CTask3 : public BaseTask {
public:
    CTask3(char *aName);
    ~CTask3();

    int Run(void);

private:
    // Task3-specific variables here
    int xCounter;
};

// Declare external pointers to instantiated tasks so other files can access them
extern CTask1* Task1;
extern CTask2* Task2;
extern CTask3* Task3;

#endif  // TASKS_HPP