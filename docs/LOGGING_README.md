# C++ Logging System

A simple and flexible logging system for C++ that provides multiple log levels, global and named logger instances, and both console and file output.

## Quick Start

```cpp
#include "Logger.hpp"

int main() {
    // Initialize the global logger
    Logger::init("logs/app.log", Logger::Level::INFO);

    // Use global logger functions
    Logger::info("Application started");
    Logger::debug("Debug information");
    Logger::warning("Warning message");
    Logger::error("Error occurred");

    // Cleanup when done
    Logger::cleanup();

    return 0;
}
```

## Usage Examples

### Global Logger Functions

```cpp
// Basic logging
Logger::info("Simple message");
Logger::debug("Debug info");
Logger::warning("Warning message");
Logger::error("Error occurred");

// Multiple arguments (variadic templates)
Logger::info("State: ", stateName, " Value: ", value);
Logger::debug("Matrix: ", matrix, " Vector: ", vector);
Logger::warning("Component: ", component, " Status: ", status);
```

### Named Logger Instances

Create named logger instances for different components:

```cpp
// In your class header
class AttitudeControl {
private:
    Logger::LoggerInstance logger;
    // ... other members
};

// In your class constructor
AttitudeControl::AttitudeControl() : logger("Attitude Control") {
    // Constructor code
}

// In your class methods
void AttitudeControl::someMethod() {
    logger.info("Starting attitude control");
    logger.debug("Current state: ", currentState);
    logger.warning("Threshold exceeded: ", threshold);
    logger.error("Control loop failed");
}
```

### Multiple Arguments

The logger supports multiple arguments for flexible message composition:

```cpp
// String concatenation
Logger::info("Processing " + std::to_string(count) + " items");

// Multiple arguments (preferred)
Logger::info("Processing ", count, " items");
Logger::debug("Position: ", x, ", ", y, ", ", z);
Logger::warning("Temperature: ", temp, "°C exceeds limit: ", limit, "°C");

// With logger instances
logger.info("Motor ", motorId, " velocity: ", velocity);
logger.debug("Matrix rotation: ", rotationMatrix);
```

## Log Levels

- **DEBUG (0)**: Detailed debugging information
- **INFO (1)**: General information about program execution
- **WARNING (2)**: Warning messages for potential issues
- **ERROR (3)**: Error messages for serious problems
- **CRITICAL (4)**: Critical errors that may prevent execution

## Configuration

### Setting Log Level

```cpp
// Set global log level
Logger::setLevel(Logger::Level::DEBUG);  // Show all messages
Logger::setLevel(Logger::Level::WARNING); // Show warnings and above only

// Get current level
Logger::Level current = Logger::getLevel();
```

### Initialization

```cpp
// Initialize with custom log file and level
Logger::init("logs/myapp.log", Logger::Level::INFO);

// Initialize with default settings
Logger::init("logs/debug.log", Logger::Level::DEBUG);
```

## Output Format

### Console Output (with colors)

```
[2024-01-15 14:30:25.123] [INFO] Application started
[2024-01-15 14:30:25.124] [DEBUG] Processing 5 items
[2024-01-15 14:30:25.125] [WARNING] Temperature: 85°C exceeds limit: 80°C
[2024-01-15 14:30:25.126] [Attitude Control] [INFO] Starting attitude control
```

### File Output (same format, no colors)

All messages are also written to the log file with timestamps and log levels.

## Best Practices

### 1. Use Named Logger Instances for Components

```cpp
// Good: Named logger for component
class MotorController {
private:
    Logger::LoggerInstance logger;
public:
    MotorController() : logger("Motor Controller") {}
    void control() {
        logger.info("Starting motor control");
        logger.debug("Current RPM: ", rpm);
    }
};
```

### 2. Use Multiple Arguments Instead of String Concatenation

```cpp
// Good: Multiple arguments
Logger::info("Processing ", count, " items in ", time, "ms");

// Avoid: String concatenation
Logger::info("Processing " + std::to_string(count) + " items in " + std::to_string(time) + "ms");
```

### 3. Set Appropriate Log Levels

```cpp
// Development
Logger::setLevel(Logger::Level::DEBUG);

// Production
Logger::setLevel(Logger::Level::INFO);

// Critical systems
Logger::setLevel(Logger::Level::WARNING);
```

### 4. Initialize Early, Cleanup Late

```cpp
int main() {
    Logger::init("logs/app.log", Logger::Level::INFO);

    // Your application code here

    Logger::cleanup();
    return 0;
}
```

## Thread Safety

The logging system is thread-safe using mutex protection for concurrent access.

## Integration Examples

### In AttitudeControl.cpp

```cpp
// Using named logger instance
logger.info("State: ", stateName);
logger.debug("Rotation matrix: ", rotationMatrix);
logger.warning("Threshold exceeded");
logger.error("Control loop failed");
```

### In Devices.cpp

```cpp
// Using global logger
Logger::info("[LabJack] Successfully Opened");
Logger::error("Failed to open LabJack");
```

### In main.cpp

```cpp
Logger::init("logs/debug.log", Logger::Level::INFO);
Logger::info("Starting Grotifer application");
// ... application code ...
Logger::info("[KILLING] Kill signal detected (E pressed)");
Logger::cleanup();
```

## Building

The logging system is header-only. Simply include `"Logger.hpp"` in your source files:

```cpp
#include "Logger.hpp"
```

No additional compilation steps or libraries required.
