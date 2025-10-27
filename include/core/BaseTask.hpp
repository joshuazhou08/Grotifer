#pragma once
#include <iostream>
#include <memory>
#include <filesystem>
#include <fstream>
#include <vector>
#include <thread>
#include <atomic>
#include <functional>
#include <string>
#include "core/LockFreeRingBuffer.hpp"

// Task state enum
enum TaskState
{
    INITIALIZING = 0
};

// -----------------------
// BaseTask class
// -----------------------
class BaseTask
{
protected:
    const char *name;
    int taskID;

    // Task timing
    double deltaTaskTime; // Task period
    double nextTaskTime;  // Time for next execution

    double timeStart;
    double timeEnd;

    // State variables for simple state machines
    int state;
    std::string stateName;
    int nextState;
    std::string nextStateName;
    int numScans;

    // for logging
    template <typename T, size_t N>
    LockFreeRingBuffer<T, N>* addQueue(const std::string& filename);

    
public:
    BaseTask(const char *name, int taskID = 0);
    virtual ~BaseTask();

    virtual int Run(void);

    int GetTaskID();

private: // For logging in separate thread
    // internal polymorphic queue interface
    struct IQueue {
        virtual ~IQueue() = default;
        virtual void flush() = 0;
    };

    template <typename T, size_t N>
    struct QueueHolder : IQueue {
        explicit QueueHolder(const std::string& filename)
        {
            // ensure the logs directory exists
            std::filesystem::create_directories("logs");

            // prepend logs/ automatically
            std::string fullPath = "logs/" + filename;
            file.open(fullPath);
            if (!file.is_open())
                throw std::runtime_error("Failed to open log file: " + fullPath);
        }

    LockFreeRingBuffer<T, N> queue;
    std::ofstream file;


        void flush() override {
            std::vector<T> batch;
            batch.reserve(256);
            while (queue.popBatch(batch)) {
                for (const auto& row : batch) {
                    for (size_t i = 0; i < row.size(); ++i)
                        file << row[i] << (i + 1 < row.size() ? ',' : '\n');
                }
            }
            file.flush();
        }
    };

    // background thread for flushing all queues of this task
    void startLogger();
    void stopLogger();
    void loggerLoop();

    std::vector<std::unique_ptr<IQueue>> queues_;
    std::thread writer_;
    std::atomic<bool> running_{false};
};


template <typename T, size_t N>
LockFreeRingBuffer<T, N>* BaseTask::addQueue(const std::string& filename) {
    auto holder = std::make_unique<QueueHolder<T, N>>(filename);
    auto* qptr = &holder->queue;
    queues_.push_back(std::move(holder));

    if (!running_) startLogger(); // ensure logger running on first add
    return qptr;
}