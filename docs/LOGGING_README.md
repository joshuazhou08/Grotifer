# C++ Logging System

This project includes a simple logging system similar to Python's logging module for C++. The logging system provides different log levels, global configuration, and both console and file output.

## Features

- **Multiple Log Levels**: DEBUG, INFO, WARNING, ERROR, CRITICAL
- **Global Configuration**: Set log level globally to control output
- **Thread Safe**: Uses mutex for thread safety
- **Console and File Output**: Logs to both console (with colors) and file
- **Timestamped**: Each log entry includes a timestamp with milliseconds
- **Configurable**: Easy to set log level and log file location

## Usage

### Basic Usage

```cpp
#include "Logger.hpp"

int main() {
    // Initialize the logger
    Logger::init("logs/app.log", Logger::Level::INFO);
    
    // Log messages at different levels
    Logger::debug("This is a debug message");
    Logger::info("This is an info message");
    Logger::warning("This is a warning message");
    Logger::error("This is an error message");
    Logger::critical("This is a critical message");
    
    // Cleanup when done
    Logger::cleanup();
    
    return 0;
}
```

### Changing Log Level

```cpp
// Set to only show WARNING and above
Logger::setLevel(Logger::Level::WARNING);

// Set to show all messages
Logger::setLevel(Logger::Level::DEBUG);

// Get current log level
Logger::Level currentLevel = Logger::getLevel();
```

### Configuration

The logging level can be configured in the `AttitudeConfig` struct in `include/Config.hpp`:

```cpp
struct AttitudeConfig {
    // ... other config ...
    
    // --- Logging Configuration --- //
    Logger::Level logLevel = Logger::Level::INFO; // Global logging level
};
```

## Log Levels

- **DEBUG (0)**: Detailed information for debugging
- **INFO (1)**: General information about program execution
- **WARNING (2)**: Warning messages for potentially problematic situations
- **ERROR (3)**: Error messages for serious problems
- **CRITICAL (4)**: Critical errors that may prevent the program from running

## Output Format

Log messages include:
- Timestamp with milliseconds: `[2024-01-15 14:30:25.123]`
- Log level: `[DEBUG]`, `[INFO]`, etc.
- Message content

Example:
```
[2024-01-15 14:30:25.123] [DEBUG] Orientation Matrix Calculated
[2024-01-15 14:30:25.124] [INFO] Starting Grotifer application
[2024-01-15 14:30:25.125] [WARNING] Kill signal detected (E pressed)
```

## Console Colors

- **DEBUG/INFO**: Normal text
- **WARNING**: Yellow text
- **ERROR/CRITICAL**: Red text

## File Output

All log messages are also written to the specified log file with the same format but without colors.

## Thread Safety

The logging system is thread-safe using a mutex to prevent concurrent access to the log file and console output.

## Integration with Existing Code

The logging system has been integrated into the existing codebase:

1. **AttitudeControl**: Replaced debug `std::cout` statements with `Logger::debug()`
2. **main.cpp**: Added application-level logging
3. **Config.hpp**: Added logging level configuration

## Example

See `src/logging_example.cpp` for a complete example of how to use the logging system.

## Building

The logging system is header-only and doesn't require additional compilation steps. Just include `"Logger.hpp"` in your source files. 