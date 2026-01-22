#include <iostream>
#include "dotenv/dotenv.h"
#include <mutex>
#pragma once

static inline void loadEnvFileOnce()
{
    static std::once_flag onceFlag;
    std::call_once(onceFlag, []()
                   {
        dotenv::init(dotenv::Preserve);
        const char* sample = std::getenv("X_VELOCITY_K_P");
        if (sample != nullptr) {
            std::cout << "[Config] Loaded .env (X_VELOCITY_K_P=" << sample << ")" << std::endl;
        } else {
            std::cout << "[Config] Loaded .env (X_VELOCITY_K_P not set)" << std::endl;
        } });
}

static inline double envOrDefault(const char *name, double defaultValue)
{
    loadEnvFileOnce();
    const char *value = std::getenv(name);
    if (value == nullptr || *value == '\0')
    {
        return defaultValue;
    }
    char *end = nullptr;
    const double parsed = std::strtod(value, &end);
    if (end == value)
    {
        return defaultValue;
    }
    return parsed;
}