#pragma once
#include "hardware/actuators/Actuator.hpp"
#include <Eigen/Dense>
#include <cstdint>
#include <set>
#include <string>

// Motor configuration parameters
struct MaxonParameters {
    // Motor identification
    char* motorName;
    uint32_t serialNo;
    
    // Controller gains
    long long unsigned int K_P;
    long long unsigned int K_I;
    double K_TUNING;
    long long unsigned int KFF_VEL;
    long long unsigned int KFF_ACC;
    long long unsigned int KP_CURR;
    long long unsigned int KI_CURR;
    
    // Encoder parameters
    unsigned int NUM_OF_PULSE_PER_REV;
    int SENSOR_POLARITY;
    
    // Motor constants
    double KT; // Torque constant [Nm/A]
    
    // Physical parameters
    double momentOfInertia; // Moment of inertia [kg*m^2]
    int maxVelocity;        // Maximum velocity [rpm]

    // Torp homing parameters for L/R Maxon Motors
    double homingVel;
    double maxAcc; 

    double offsetPos;
    double offsetPosLim;
    double startPosLim;

};

class MaxonMotor : public Actuator {
public:
    // Constructor - opens motor by serial number and initializes
    // The motor will scan USB ports and open the one matching params.serialNo

    // X, Y, Z Maxon Motors
    MaxonMotor(MaxonParameters& params, Axis axis);

    // L & R Torp Maxon Motors
    MaxonMotor(MaxonParameters& params, Side side);
    
    ~MaxonMotor() override;
    
    // Actuator interface implementation
    bool isOpen() const override { return isOpen_; }
    void applyTorque(const Eigen::Vector3d& torqueCmd, double deltaT) override;
    Eigen::Vector3d getTorque() const override;
    double getSpeed() const override { return getVelocity(); } // Returns motor velocity in rpm
    double getMotPos() const override { return getPosition(); } // Returns motor position *** UNITS? 

    // MaxonMotor-specific getters
    double getVelocity() const; // Returns actual velocity in rpm
    double getPosition() const; // Returns motor position

    // MaxonMotor homing position getters
    double getOffsetPos() { return params.offsetPos; }    // Returns offsetPos
    double getOffsetPosLim() { return params.offsetPosLim; }  // Returns offsetPosLim
    double getStartPosLim() { return params.startPosLim; }   // Returns startPosLim
    
    // MaxonMotor homing movement getters
    double getHomingVel() { return params.homingVel; }
    double getMaxAcc() { return params.maxAcc; }

    // MaxonMotor stop function
    
    void haltMotion();
    bool setVelocity(int velocityRPM);

private:
    void* openMotorBySerialNumber(uint32_t targetSerialNo);
    void initSettings(const MaxonParameters& params);
    void clearFault();
    void setEnableState();
    void setDisableState();
    bool setVelocityCommand(int velocityRPM);

    
    // Static port tracking to prevent multiple motors from opening the same port
    static std::set<std::string> openPorts_;
    
    // Hardware interface
    void* handle_;
    uint32_t serialNo_;
    const char* motorName_;
    std::string portName_;     // Track which port this motor is using
    unsigned int nodeID_ = 1;
    unsigned int baudrate_ = 1000000;
    unsigned int timeout_ = 500;
    mutable unsigned int errorCode_ = 0;
    
    // Motor parameters
    double torqueConstant_;    // KT - torque constant [Nm/A]
    double momentOfInertia_;   // Moment of inertia [kg*m^2]
    int maxVelocity_;          // Maximum velocity [rpm]
    double motorPos_;          // Motor position as measured by Maxon controller
    
    // State tracking
    double currentVelocity_ = 0.0;  // Accumulated velocity in rpm (double for precise integration)
    int commandedVelocity_ = 0; // Last commanded velocity in rpm (sent to motor as int)

    MaxonParameters params;
    
    bool isOpen_ = false;
};

