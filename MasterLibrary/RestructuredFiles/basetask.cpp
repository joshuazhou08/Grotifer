#include "basetask.hpp"
#include <cstring>

// BaseTask implementation

BaseTask::BaseTask(char* taskName, unsigned int taskID) :
    p_taskID(taskID), p_deltaTaskTime(0.0), p_nextTaskTime(0.0), 
    p_state(0), p_nextState(0), p_runSuccess(false), p_numScans(0),
    p_auditTrailDataFile(nullptr)
{
    if (taskName) {
        p_taskName = new char[strlen(taskName) +1];
        strcpy(p_taskName, taskName);
    } else {
        p_taskName = nullptr;
    }
}

BaseTask::~BaseTask() {
    if (p_taskName) {
        delete[] p_taskName;
        p_taskName = nullptr;
    }
}

// TListElement implementation

TListElement::TListElement(BaseTask* taskPtr) : task(taskPtr), next(nullptr) {}

TListElement::~TListElement() {
    // task pointer deleted in main(), not here
    task = nullptr;
    next = nullptr;
}

// TList implementation

TList::TList(const char* listName) :
    first(nullptr), last(nullptr), current(nullptr), length(0)

{
    if  (listName) {
        name = new char[strlen(listName) + 1];
        strcpy(name, listName);
    } else {
        name = nullptr;
    }
}

TList::~TList() {
    TListElement* elem = first;
    while (elem != nullptr) {
        TListElement* nextElem = elem->next;
        delete elem;
        elem = nextElem;
    }
    first = nullptr;
    last = nullptr;
    current = nullptr;
    length = 0;
    if (name) {
        delete[] name;
        name = nullptr;
    }
}

// TList append
void TList::Append(BaseTask* task) {
    TListElement* elem = new TListElement(task);
    if(last != nullptr) {
        last->next = elem;
        last = elem;
    } else {
        first = elem;
        last = elem;
    }
    length++;
}

BaseTask* TList::NextItem() {
    if (current == nullptr) {
        current = first;
    } else {
        current = current->next;
    }

    if (current != nullptr) {
        return current->task;
    } else {
        current = nullptr;
        return nullptr;
    }
}

bool TList::IsEmpty() const {
    return (length == 0);
}

int TList::Length() const {
    return length;
}