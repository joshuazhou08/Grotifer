#pragma once

#include <iostream>
#include <string>
#include <sstream>
#include <chrono>
#include <iomanip>
#include <fstream>
#include <mutex>
#include <filesystem>
#include <memory>

namespace Logger {

// Log levels enum
enum class Level {
    DEBUG = 0,
    INFO = 1,
    WARNING = 2,
    ERROR = 3,
    CRITICAL = 4
};

// Logger class - each instance has its own level and file
class Logger {
private:
    Level logLevel;
    std::ofstream logFile;
    std::mutex logMutex;
    std::string name;

public:
    // Constructor
    Logger(const std::string& name = "default", 
           const std::string& filename = "logs/app.log", 
           Level level = Level::INFO);
    
    // Destructor
    ~Logger();
    
    // Set the log level for this logger
    void setLevel(Level level);
    
    // Get the current log level
    Level getLevel() const;
    
    // Get logger name
    std::string getName() const;
    
    // Main logging function
    void log(Level level, const std::string& message);
    
    // Convenience functions for different log levels
    void debug(const std::string& message);
    void info(const std::string& message);
    void warning(const std::string& message);
    void error(const std::string& message);
    void critical(const std::string& message);
    
    // Static helper functions for backward compatibility
    static std::string getLevelName(Level level);
    static std::string getCurrentTimestamp();
};

// Global default logger for backward compatibility
extern std::unique_ptr<Logger> defaultLogger;

// Initialize the default logger
void init(const std::string& filename = "logs/app.log", Level level = Level::INFO);

// Set the global default logger level
void setLevel(Level level);

// Get the current global default logger level
Level getLevel();

// Convenience functions using the default logger
void debug(const std::string& message);
void info(const std::string& message);
void warning(const std::string& message);
void error(const std::string& message);
void critical(const std::string& message);

// Variadic convenience functions using the default logger
template<typename... Args>
void debug(Args&&... args);

template<typename... Args>
void info(Args&&... args);

template<typename... Args>
void warning(Args&&... args);

template<typename... Args>
void error(Args&&... args);

template<typename... Args>
void critical(Args&&... args);

// Cleanup function
void cleanup();

} // namespace Logger

// Implementation
namespace Logger {

inline std::unique_ptr<Logger> defaultLogger = nullptr;

inline Logger::Logger(const std::string& name, const std::string& filename, Level level)
    : logLevel(level), name(name) {
    
    // Create logs directory if it doesn't exist
    size_t lastSlash = filename.find_last_of('/');
    if (lastSlash != std::string::npos) {
        std::string dir = filename.substr(0, lastSlash);
        std::filesystem::create_directories(dir);
    }
    
    logFile.open(filename, std::ios::out | std::ios::app);
    if (!logFile.is_open()) {
        std::cerr << "Failed to open log file: " << filename << std::endl;
    }
}

inline Logger::~Logger() {
    std::lock_guard<std::mutex> lock(logMutex);
    if (logFile.is_open()) {
        logFile.close();
    }
}

inline void Logger::setLevel(Level level) {
    std::lock_guard<std::mutex> lock(logMutex);
    logLevel = level;
}

inline Level Logger::getLevel() const {
    return logLevel;
}

inline std::string Logger::getName() const {
    return name;
}

inline std::string Logger::getLevelName(Level level) {
    switch (level) {
        case Level::DEBUG: return "DEBUG";
        case Level::INFO: return "INFO";
        case Level::WARNING: return "WARNING";
        case Level::ERROR: return "ERROR";
        case Level::CRITICAL: return "CRITICAL";
        default: return "UNKNOWN";
    }
}

inline std::string Logger::getCurrentTimestamp() {
    auto now = std::chrono::system_clock::now();
    auto time_t = std::chrono::system_clock::to_time_t(now);
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        now.time_since_epoch()) % 1000;
    
    std::stringstream ss;
    ss << std::put_time(std::localtime(&time_t), "%Y-%m-%d %H:%M:%S");
    ss << '.' << std::setfill('0') << std::setw(3) << ms.count();
    return ss.str();
}

inline void Logger::log(Level level, const std::string& message) {
    if (level < logLevel) {
        return;
    }
    
    std::lock_guard<std::mutex> lock(logMutex);
    
    std::stringstream ss;
    ss << "[" << getCurrentTimestamp() << "] [" << name << "] [" << getLevelName(level) << "] " << message;
    
    // Output to console
    switch (level) {
        case Level::DEBUG:
        case Level::INFO:
            std::cout << ss.str() << std::endl;
            break;
        case Level::WARNING:
            std::cout << "\033[33m" << ss.str() << "\033[0m" << std::endl; // Yellow
            break;
        case Level::ERROR:
        case Level::CRITICAL:
            std::cerr << "\033[31m" << ss.str() << "\033[0m" << std::endl; // Red
            break;
    }
    
    // Output to file
    if (logFile.is_open()) {
        logFile << ss.str() << std::endl;
        logFile.flush();
    }
}

inline void Logger::debug(const std::string& message) {
    log(Level::DEBUG, message);
}

inline void Logger::info(const std::string& message) {
    log(Level::INFO, message);
}

inline void Logger::warning(const std::string& message) {
    log(Level::WARNING, message);
}

inline void Logger::error(const std::string& message) {
    log(Level::ERROR, message);
}

inline void Logger::critical(const std::string& message) {
    log(Level::CRITICAL, message);
}

// Global default logger functions
inline void init(const std::string& filename, Level level) {
    defaultLogger = std::make_unique<Logger>("default", filename, level);
}

inline void setLevel(Level level) {
    if (defaultLogger) {
        defaultLogger->setLevel(level);
    }
}

inline Level getLevel() {
    return defaultLogger ? defaultLogger->getLevel() : Level::INFO;
}

inline void debug(const std::string& message) {
    if (defaultLogger) {
        defaultLogger->debug(message);
    }
}

inline void info(const std::string& message) {
    if (defaultLogger) {
        defaultLogger->info(message);
    }
}

inline void warning(const std::string& message) {
    if (defaultLogger) {
        defaultLogger->warning(message);
    }
}

inline void error(const std::string& message) {
    if (defaultLogger) {
        defaultLogger->error(message);
    }
}

inline void critical(const std::string& message) {
    if (defaultLogger) {
        defaultLogger->critical(message);
    }
}

// Variadic template implementations
template<typename... Args>
inline void debug(Args&&... args) {
    if (defaultLogger) {
        std::stringstream ss;
        (ss << ... << std::forward<Args>(args));
        defaultLogger->debug(ss.str());
    }
}

template<typename... Args>
inline void info(Args&&... args) {
    if (defaultLogger) {
        std::stringstream ss;
        (ss << ... << std::forward<Args>(args));
        defaultLogger->info(ss.str());
    }
}

template<typename... Args>
inline void warning(Args&&... args) {
    if (defaultLogger) {
        std::stringstream ss;
        (ss << ... << std::forward<Args>(args));
        defaultLogger->warning(ss.str());
    }
}

template<typename... Args>
inline void error(Args&&... args) {
    if (defaultLogger) {
        std::stringstream ss;
        (ss << ... << std::forward<Args>(args));
        defaultLogger->error(ss.str());
    }
}

template<typename... Args>
inline void critical(Args&&... args) {
    if (defaultLogger) {
        std::stringstream ss;
        (ss << ... << std::forward<Args>(args));
        defaultLogger->critical(ss.str());
    }
}

inline void cleanup() {
    defaultLogger.reset();
}

} // namespace Logger 