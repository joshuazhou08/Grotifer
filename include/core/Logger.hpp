#pragma once

#include <iostream>
#include <string>
#include <sstream>
#include <chrono>
#include <iomanip>
#include <fstream>
#include <mutex>
#include <filesystem>

namespace Logger
{

    // Log levels enum
    enum class Level
    {
        DEBUG = 0,
        INFO = 1,
        WARNING = 2,
        ERROR = 3,
        CRITICAL = 4
    };

    // Logger class for named instances
    class LoggerInstance
    {
    public:
        LoggerInstance(const std::string& name);
        
        // Main logging function
        void log(Level level, const std::string &message);
        
        // Convenience functions
        void debug(const std::string &message);
        void info(const std::string &message);
        void warning(const std::string &message);
        void error(const std::string &message);
        void critical(const std::string &message);
        
        // Variadic convenience functions
        template <typename... Args>
        void debug(Args &&...args);
        
        template <typename... Args>
        void info(Args &&...args);
        
        template <typename... Args>
        void warning(Args &&...args);
        
        template <typename... Args>
        void error(Args &&...args);
        
        template <typename... Args>
        void critical(Args &&...args);
        
    private:
        std::string name;
    };

    // Global logger state (for backward compatibility)
    extern Level currentLevel;
    extern std::ofstream logFile;
    extern std::mutex logMutex;
    extern std::string loggerName;

    // Initialize the logger
    void init(const std::string &filename = "logs/app.log", Level level = Level::INFO);

    // Set the logger level
    void setLevel(Level level);

    // Get the current logger level
    Level getLevel();

    // Helper functions
    std::string getLevelName(Level level);
    std::string getCurrentTimestamp();

    // Main logging function (global)
    void log(Level level, const std::string &message);

    // Convenience functions (global)
    void debug(const std::string &message);
    void info(const std::string &message);
    void warning(const std::string &message);
    void error(const std::string &message);
    void critical(const std::string &message);

    // Variadic convenience functions (global)
    template <typename... Args>
    void debug(Args &&...args);

    template <typename... Args>
    void info(Args &&...args);

    template <typename... Args>
    void warning(Args &&...args);

    template <typename... Args>
    void error(Args &&...args);

    template <typename... Args>
    void critical(Args &&...args);

    // Cleanup function
    void cleanup();

} // namespace Logger

// Implementation
namespace Logger
{

    inline Level currentLevel = Level::INFO;
    inline std::ofstream logFile;
    inline std::mutex logMutex;
    inline std::string loggerName = "default";

    inline void init(const std::string &filename, Level level)
    {
        std::lock_guard<std::mutex> lock(logMutex);

        // Create logs directory if it doesn't exist
        size_t lastSlash = filename.find_last_of('/');
        if (lastSlash != std::string::npos)
        {
            std::string dir = filename.substr(0, lastSlash);
            std::filesystem::create_directories(dir);
        }

        logFile.open(filename, std::ios::out | std::ios::app);
        if (!logFile.is_open())
        {
            std::cerr << "Failed to open log file: " << filename << std::endl;
        }

        currentLevel = level;
    }

    inline void setLevel(Level level)
    {
        std::lock_guard<std::mutex> lock(logMutex);
        currentLevel = level;
    }

    inline Level getLevel()
    {
        return currentLevel;
    }

    inline std::string getLevelName(Level level)
    {
        switch (level)
        {
        case Level::DEBUG:
            return "DEBUG";
        case Level::INFO:
            return "INFO";
        case Level::WARNING:
            return "WARNING";
        case Level::ERROR:
            return "ERROR";
        case Level::CRITICAL:
            return "CRITICAL";
        default:
            return "UNKNOWN";
        }
    }

    inline std::string getCurrentTimestamp()
    {
        auto now = std::chrono::system_clock::now();
        auto time_t = std::chrono::system_clock::to_time_t(now);
        auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                      now.time_since_epoch()) %
                  1000;

        std::stringstream ss;
        ss << std::put_time(std::localtime(&time_t), "%Y-%m-%d %H:%M:%S");
        ss << '.' << std::setfill('0') << std::setw(3) << ms.count();
        return ss.str();
    }

    inline void log(Level level, const std::string &message)
    {
        if (level < currentLevel)
        {
            return;
        }

        std::lock_guard<std::mutex> lock(logMutex);

        std::stringstream ss;
        ss << "[" << getCurrentTimestamp() << "] [" << loggerName << "] [" << getLevelName(level) << "] " << message;

        // Output to console
        switch (level)
        {
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
        if (logFile.is_open())
        {
            logFile << ss.str() << std::endl;
            logFile.flush();
        }
    }

    inline void debug(const std::string &message)
    {
        log(Level::DEBUG, message);
    }

    inline void info(const std::string &message)
    {
        log(Level::INFO, message);
    }

    inline void warning(const std::string &message)
    {
        log(Level::WARNING, message);
    }

    inline void error(const std::string &message)
    {
        log(Level::ERROR, message);
    }

    inline void critical(const std::string &message)
    {
        log(Level::CRITICAL, message);
    }

    // Variadic template implementations
    template <typename... Args>
    inline void debug(Args &&...args)
    {
        std::stringstream ss;
        (ss << ... << std::forward<Args>(args));
        log(Level::DEBUG, ss.str());
    }

    template <typename... Args>
    inline void info(Args &&...args)
    {
        std::stringstream ss;
        (ss << ... << std::forward<Args>(args));
        log(Level::INFO, ss.str());
    }

    template <typename... Args>
    inline void warning(Args &&...args)
    {
        std::stringstream ss;
        (ss << ... << std::forward<Args>(args));
        log(Level::WARNING, ss.str());
    }

    template <typename... Args>
    inline void error(Args &&...args)
    {
        std::stringstream ss;
        (ss << ... << std::forward<Args>(args));
        log(Level::ERROR, ss.str());
    }

    template <typename... Args>
    inline void critical(Args &&...args)
    {
        std::stringstream ss;
        (ss << ... << std::forward<Args>(args));
        log(Level::CRITICAL, ss.str());
    }

    inline void cleanup()
    {
        std::lock_guard<std::mutex> lock(logMutex);
        if (logFile.is_open())
        {
            logFile.close();
        }
    }

    // LoggerInstance implementation
    inline LoggerInstance::LoggerInstance(const std::string& name) : name(name) {}

    inline void LoggerInstance::log(Level level, const std::string &message)
    {
        if (level < currentLevel)
        {
            return;
        }

        std::lock_guard<std::mutex> lock(logMutex);

        std::stringstream ss;
        ss << "[" << getCurrentTimestamp() << "] [" << name << "] [" << getLevelName(level) << "] " << message;

        // Output to console
        switch (level)
        {
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
        if (logFile.is_open())
        {
            logFile << ss.str() << std::endl;
            logFile.flush();
        }
    }

    inline void LoggerInstance::debug(const std::string &message)
    {
        log(Level::DEBUG, message);
    }

    inline void LoggerInstance::info(const std::string &message)
    {
        log(Level::INFO, message);
    }

    inline void LoggerInstance::warning(const std::string &message)
    {
        log(Level::WARNING, message);
    }

    inline void LoggerInstance::error(const std::string &message)
    {
        log(Level::ERROR, message);
    }

    inline void LoggerInstance::critical(const std::string &message)
    {
        log(Level::CRITICAL, message);
    }

    // LoggerInstance variadic template implementations
    template <typename... Args>
    inline void LoggerInstance::debug(Args &&...args)
    {
        std::stringstream ss;
        (ss << ... << std::forward<Args>(args));
        log(Level::DEBUG, ss.str());
    }

    template <typename... Args>
    inline void LoggerInstance::info(Args &&...args)
    {
        std::stringstream ss;
        (ss << ... << std::forward<Args>(args));
        log(Level::INFO, ss.str());
    }

    template <typename... Args>
    inline void LoggerInstance::warning(Args &&...args)
    {
        std::stringstream ss;
        (ss << ... << std::forward<Args>(args));
        log(Level::WARNING, ss.str());
    }

    template <typename... Args>
    inline void LoggerInstance::error(Args &&...args)
    {
        std::stringstream ss;
        (ss << ... << std::forward<Args>(args));
        log(Level::ERROR, ss.str());
    }

    template <typename... Args>
    inline void LoggerInstance::critical(Args &&...args)
    {
        std::stringstream ss;
        (ss << ... << std::forward<Args>(args));
        log(Level::CRITICAL, ss.str());
    }

} // namespace Logger