#pragma once
#include <cstdint>

// Forward declaration of LabJack types from u6.h
typedef void* HANDLE;
struct U6_CALIBRATION_INFORMATION;
typedef struct U6_CALIBRATION_INFORMATION u6CalibrationInfo;

// LabJack U6 hardware interface for reading analog inputs, timers, and counters
// Provides low-level access to LabJack U6 DAQ device for sensors
class LabJackU6 {
public:
    // Constructor - opens LabJack U6 device and initializes with timer/counter pin offset
    LabJackU6(long tcPinOffset = 0);
    ~LabJackU6();

    // Check if the LabJack is open and operational
    bool isOpen() const { return handle_ != nullptr; }

    // Configure timers and counters for quadrature encoder reading
    void configTimersCounters();

    // Get counts value from a specific timer channel (for encoders)
    // Returns signed 32-bit count value
    long getCountsValueAtTimer(unsigned int timerChannel);

    // Get counter value from a specific counter channel
    double getCounterValueAtCounter(unsigned int counterChannel);

    // Read raw voltage from an analog input channel
    double getRawVoltageAtChannel(long channel);

    // Read averaged voltage from an analog input channel (5 samples)
    double getAveVoltageAtChannel(long channel);

private:
    // Open the LabJack U6 device
    bool openConnection();
    
    void getCalibrationInfo();

    // Hardware handle and calibration
    HANDLE handle_ = nullptr;
    u6CalibrationInfo* caliInfo_ = nullptr;

    // Timer and counter configuration
    static const unsigned int numTimers_ = 4;
    static const unsigned int numCounters_ = 2;
    long tcPinOffset_;
    long timerClockBaseIndex_;
    long timerClockDivisor_;

    // Timer arrays
    long enableTimers_[numTimers_];
    long timerModes_[numTimers_];
    double timerValues_[numTimers_];
    long readTimers_[numTimers_];
    long updateResetTimers_[numTimers_];

    // Counter arrays
    long enableCounters_[numCounters_];
    long readCounters_[numCounters_];
    long resetCounters_[numCounters_];
    double counterValues_[numCounters_];
};

class LJEncoder {

public:
    LJEncoder(LabJackU6& labjack, uint16_t cpr, unsigned int tcOrder);
    
    ~LJEncoder(); 

    // Get angular position in degrees from encoder
    double getAngularPosDeg();

    // Get counter/index
    uint16_t getIndexCounterSignal();

    // Get index flag when index is reached
    bool getIndexFlag();

protected:
    LabJackU6& labJack_;
    uint16_t quad_cpr = 1;
    double pos = 0;
    uint16_t counterVal = 0;
    bool indexFlag = false;
    unsigned int timerChannel = 0;
    unsigned int counterChannel = 0;

};