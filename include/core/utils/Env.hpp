#include <iostream>
#include "dotenv/dotenv.h"
#include <cctype>
#include <cstring>
#include <mutex>
#include <string>
#pragma once

static inline void loadEnvFileOnce()
{
    static std::once_flag onceFlag;
    std::call_once(onceFlag, []() {
        dotenv::init(dotenv::Preserve);
        const char* sample = std::getenv("X_VELOCITY_K_P");
        if (sample != nullptr) {
            std::cout << "[Config] Loaded .env (X_VELOCITY_K_P=" << sample << ")" << std::endl;
        } else {
            std::cout << "[Config] Loaded .env (X_VELOCITY_K_P not set)" << std::endl;
        }
    });
}

static inline double envOrDefault(const char* name, double defaultValue)
{
    loadEnvFileOnce();
    const char* value = std::getenv(name);
    if (value == nullptr || *value == '\0') {
        return defaultValue;
    }
    char* end = nullptr;
    const double parsed = std::strtod(value, &end);
    if (end == value) {
        return defaultValue;
    }
    return parsed;
}

static inline double envOrDefault(const char* name, int defaultValue)
{
    return envOrDefault(name, static_cast<double>(defaultValue));
}

static inline bool envOrDefault(const char* name, bool defaultValue)
{
    loadEnvFileOnce();
    const char* value = std::getenv(name);
    if (value == nullptr || *value == '\0') {
        return defaultValue;
    }
    std::string normalized;
    normalized.reserve(std::strlen(value));
    for (const char* ptr = value; *ptr != '\0'; ++ptr) {
        if (!std::isspace(static_cast<unsigned char>(*ptr))) {
            normalized.push_back(static_cast<char>(std::tolower(static_cast<unsigned char>(*ptr))));
        }
    }
    if (normalized.empty()) {
        return defaultValue;
    }
    if (normalized == "1" || normalized == "true" || normalized == "t" || normalized == "yes" ||
        normalized == "y" || normalized == "on") {
        return true;
    }
    if (normalized == "0" || normalized == "false" || normalized == "f" || normalized == "no" ||
        normalized == "n" || normalized == "off") {
        return false;
    }
    return defaultValue;
}
