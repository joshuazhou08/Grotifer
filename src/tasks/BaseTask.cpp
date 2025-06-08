#include "BaseTask.hpp"

// -----------------------
// BaseTask implementation
// -----------------------

BaseTask::BaseTask(const char *name, int taskID)
    : name(name),
      taskID(taskID),
      deltaTaskTime(0.0),
      nextTaskTime(0.0),
      state(0),
      nextState(-1),
      numScans(0)
{
}

BaseTask::~BaseTask()
{
    std::cout << "Concluding " << name << " Task" << std::endl;
}

int BaseTask::Run() { return 0; }

TaskNode::TaskNode(std::unique_ptr<BaseTask> t) : task(std::move(t)), next(nullptr)
{
}

TaskList::~TaskList()
{
    if (!tail)
        return;

    TaskNode *head = tail->next;
    tail->next = nullptr; // break the cycle

    while (head)
    {
        TaskNode *temp = head;
        head = head->next;
        delete temp;
    }
}

void TaskList::insert(std::unique_ptr<BaseTask> t)
{
    TaskNode *newNode = new TaskNode(std::move(t));

    if (!tail)
    {
        tail = newNode;
        tail->next = tail; // circular
    }
    else
    {
        newNode->next = tail->next;
        tail->next = newNode;
        tail = newNode;
    }
}