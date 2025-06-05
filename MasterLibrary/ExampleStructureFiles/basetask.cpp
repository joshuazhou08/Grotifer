#include "basetask.hpp"
#include <iostream>
#include <cstring>

// -----------------------
// BaseTask implementation
// -----------------------

BaseTask::BaseTask(char* aName, int taskID)
    : TaskID(taskID),
      DeltaTaskTime(0.0),
      NextTaskTime(0.0),
      State(0),
      NextState(-1),
      RunEntry(0),
      NumScans(0)
{
    if (aName) {
        Name = new char[strlen(aName) + 1];
        strcpy(Name, aName);
    } else {
        Name = nullptr;
    }
}

BaseTask::~BaseTask() {
    if (Name) delete[] Name;
}

int BaseTask::Run(void) {
    std::cerr << "Error: BaseTask Run() called directly for task: "
              << (Name ? Name : "Unnamed") << std::endl;
    return -1;
}

// -----------------------
// TListElement implementation
// -----------------------

TListElement::TListElement(BaseTask* ItemPointer)
    : Item(ItemPointer), Next(nullptr) {}

TListElement::~TListElement() {}

// -----------------------
// TList implementation
// -----------------------

TList::TList(const char* listName)
    : First(nullptr), Last(nullptr), Current(nullptr), Length(0)
{
    if (listName) {
        Name = new char[strlen(listName) + 1];
        strcpy(Name, listName);
    } else {
        Name = nullptr;
    }
}

TList::~TList() {
    // Delete all list elements, but NOT the BaseTask pointers
    TListElement* elem = First;
    while (elem) {
        TListElement* next = elem->Next;
        delete elem;
        elem = next;
    }
    if (Name) delete[] Name;
}

void TList::Append(BaseTask* Item) {
    TListElement* newElem = new TListElement(Item);
    if (Last) {
        Last->Next = newElem;
        Last = newElem;
    } else {
        First = Last = newElem;
    }
    Length++;
}

BaseTask* TList::NextItem() {
    if (!Current) Current = First;
    else Current = Current->Next;

    if (Current) return Current->Item;
    else return nullptr;
}

int TList::IsEmpty() const {
    return Length == 0;
}

int TList::HowLong() const {
    return Length;
}
