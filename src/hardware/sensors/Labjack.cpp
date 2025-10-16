#include "hardware/sensors/Labjack.hpp"
#include "../../external/labjack/u6.h"
#include <iostream>
#include <cstdint>

using namespace std;

// Constructor - opens LabJack U6 device and initializes with timer/counter pin offset
LabJackU6::LabJackU6(long tcPinOffset)
    : handle_(nullptr),
      tcPinOffset_(tcPinOffset),
      caliInfo_(nullptr)
{
    // Allocate calibration info
    caliInfo_ = new u6CalibrationInfo();
    
    // Open the connection
    if (openConnection()) {
        // Get calibration information from device
        getCalibrationInfo();
        
        // Configure timers and counters
        configTimersCounters();
    }
}

LabJackU6::~LabJackU6() {
    // Cleanup calibration info
    if (caliInfo_ != nullptr) {
        delete caliInfo_;
        caliInfo_ = nullptr;
    }
    
    // Cleanup - could close handle here if needed
    if (handle_ != nullptr) {
        // Note: LabJack library doesn't provide a close function for USB connections
        // The handle will be cleaned up automatically
    }
}

// Open the LabJack U6 device
bool LabJackU6::openConnection() {
    handle_ = openUSBConnection(-1);
    if (handle_ == NULL) {
        cerr << "[LabJackU6] Failed to open LabJack U6 device" << endl;
        handle_ = nullptr;
        return false;
    }
    
    cout << "[LabJackU6] Successfully opened LabJack U6" << endl;
    return true;
}

// Get calibration information from the device
void LabJackU6::getCalibrationInfo() {
    long error = ::getCalibrationInfo(handle_, caliInfo_);
    if (error < 0) {
        cerr << "[LabJackU6] Failed to get calibration info, error code: " << error << endl;
    }
}

// Configure timers and counters for quadrature encoder reading
void LabJackU6::configTimersCounters() {
    long error = 0;
    
    // Enable all timers and counters; Set timers to quadrature mode; Reset and disable reset
    for (int i = 0; i < numTimers_; i++) {
        enableTimers_[i] = 1;           // Enable Timer-i
        timerModes_[i] = LJ_tmQUAD;     // Timer-i is in quadrature mode
        timerValues_[i] = 0;            // Initialize reading
        readTimers_[i] = 1;             // Enable read Timers
        updateResetTimers_[i] = 1;      // Reset Timer-i
    }
    
    for (int k = 0; k < numCounters_; k++) {
        enableCounters_[k] = 1;         // Enable Counter-k
        readCounters_[k] = 1;           // Enable read Counter-k
        resetCounters_[k] = 1;          // Reset Counter-k
    }
    
    // Configure the LabJack timers and counters
    error = eTCConfig(handle_, enableTimers_, enableCounters_, tcPinOffset_, 
                      LJ_tc48MHZ, 0, timerModes_, timerValues_, 0, 0);
    if (error < 0) {
        cerr << "[LabJackU6] Failed to config timers/counters, error code: " << error << endl;
    }
    
    // Reset the LabJack
    error = eTCValues(handle_, readTimers_, updateResetTimers_, readCounters_, 
                      resetCounters_, timerValues_, counterValues_, 0, 0);
    if (error < 0) {
        cerr << "[LabJackU6] Failed to reset timers/counters, error code: " << error << endl;
    }
    
    // Disable reset
    for (int i = 0; i < numTimers_; i++) {
        updateResetTimers_[i] = 0;      // Disable reset Timer-i
    }
    
    for (int k = 0; k < numCounters_; k++) {
        resetCounters_[k] = 0;          // Disable reset Counter-k
    }
}

// Get counts value from a specific timer channel (for encoders)
long LabJackU6::getCountsValueAtTimer(unsigned int timerChannel) {
    // Check the validity of the requested timer
    unsigned int validTimerChannel = 0;
    if (timerChannel >= numTimers_) {
        cerr << "[LabJackU6] Invalid timer channel " << timerChannel 
             << ". Using Timer 0 instead." << endl;
        validTimerChannel = 0;
    } else {
        validTimerChannel = timerChannel;
    }
    
    // Read all timers and counters
    long error = eTCValues(handle_, readTimers_, updateResetTimers_, readCounters_, 
                          resetCounters_, timerValues_, counterValues_, 0, 0);
    if (error < 0) {
        cerr << "[LabJackU6] Failed to read timer, error code: " << error << endl;
    }
    
    // Extract the value and handle 32-bit signed conversion
    long countVal = 0;
    if (timerValues_[validTimerChannel] >= 2147483648.0) {
        countVal = (int32_t)(-1) * (4294967296 - timerValues_[validTimerChannel]);
    } else {
        countVal = (int32_t)timerValues_[validTimerChannel];
    }
    
    return countVal;
}

// Get counter value from a specific counter channel
double LabJackU6::getCounterValueAtCounter(unsigned int counterChannel) {
    // Check the validity of the requested counter
    unsigned int validCounterChannel = 0;
    if (counterChannel >= numCounters_) {
        cerr << "[LabJackU6] Invalid counter channel " << counterChannel 
             << ". Using Counter 0 instead." << endl;
        validCounterChannel = 0;
    } else {
        validCounterChannel = counterChannel;
    }
    
    // Read all timers and counters
    long error = eTCValues(handle_, readTimers_, updateResetTimers_, readCounters_, 
                          resetCounters_, timerValues_, counterValues_, 0, 0);
    if (error < 0) {
        cerr << "[LabJackU6] Failed to read counter, error code: " << error << endl;
    }
    
    return counterValues_[validCounterChannel];
}

// Read raw voltage from an analog input channel
double LabJackU6::getRawVoltageAtChannel(long channel) {
    double voltage = 0.0;
    long error = eAIN(handle_, caliInfo_, channel, 15, &voltage, LJ_rgBIP10V, 0, 0, 0, 0, 0);
    if (error != 0) {
        cerr << "[LabJackU6] Failed to read voltage from channel " << channel 
             << ", error code: " << error << endl;
    }
    return voltage;
}

// Read averaged voltage from an analog input channel (5 samples)
double LabJackU6::getAveVoltageAtChannel(long channel) {
    double aveVoltage = 0.0;
    double tempVoltage = 0.0;
    double sumVoltage = 0.0;
    const int numReads = 5;  // Number of reads to average
    
    for (int i = 0; i < numReads; i++) {
        long error = eAIN(handle_, caliInfo_, channel, 15, &tempVoltage, 
                         LJ_rgBIP10V, 0, 0, 0, 0, 0);
        if (error != 0) {
            cerr << "[LabJackU6] Failed to read voltage from channel " << channel 
                 << ", error code: " << error << endl;
        }
        sumVoltage += tempVoltage;
    }
    
    aveVoltage = sumVoltage / numReads;
    return aveVoltage;
}

