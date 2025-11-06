#include "hardware/actuators/MaxonMotor.hpp"
#include "hardware/actuators/Definitions.h"
#include <iostream>
#include <cmath>
#include <cstring>

using namespace std;
using namespace Eigen;

// Define static member for tracking open ports across all motor instances
set<string> MaxonMotor::openPorts_;

// Private method to open motor by scanning USB ports for matching serial number
void* MaxonMotor::openMotorBySerialNumber(uint32_t targetSerialNo) {
    // EPOS4 connection parameters (mutable buffers for old C API)
    char deviceName[] = "EPOS4";
    char protocolStackName[] = "MAXON SERIAL V2";
    char interfaceName[] = "USB";
    unsigned int nodeID = 1;
    unsigned int errorCode = 0;
    
    // Reset port name selection
    VCS_ResetPortNameSelection(deviceName, protocolStackName, interfaceName, &errorCode);
    
    // Count available ports (matching old code behavior)
    const unsigned int MAX_STR_SIZE = 100;
    char portNameTemp[MAX_STR_SIZE];
    int endOfSelection = false;
    unsigned int numAvailablePorts = 0;
    
    // Get first port
    if (!VCS_GetPortNameSelection(deviceName, protocolStackName, interfaceName, true, 
                                   portNameTemp, MAX_STR_SIZE, &endOfSelection, &errorCode)) {
        cerr << "[MaxonMotor] Failed to scan for USB ports" << endl;
        return nullptr;
    }
    numAvailablePorts++;
    
    // Count remaining ports
    while (!endOfSelection) {
        numAvailablePorts++;
        VCS_GetPortNameSelection(deviceName, protocolStackName, interfaceName, false,
                                portNameTemp, MAX_STR_SIZE, &endOfSelection, &errorCode);
    }
    
    // Try each available port using constructed names (USB0, USB1, etc.)
    // This matches the old code behavior where port names are constructed as "USB" + index
    for (unsigned int i = 0; i < numAvailablePorts; i++) {
        string portName = string("USB") + to_string(i);
        
        // Skip this port if it's already open by another motor instance
        if (openPorts_.find(portName) != openPorts_.end()) {
            continue;  // Port already in use, skip it
        }
        
        // Try to open this port
        // Use &portName[0] to get non-const char* for old C API (same as old code's &availPortNameList[i][0])
        void* handle = VCS_OpenDevice(deviceName, protocolStackName, interfaceName, 
                                      &portName[0], &errorCode);
        
        if (handle != nullptr) {
            // Read serial number from this device
            uint32_t serialNo = 0;
            unsigned int numBytesToRead = 4;
            unsigned int numBytesRead = 0;
            
            if (VCS_GetObject(handle, nodeID, 0x1018, 0x04, &serialNo, 
                             numBytesToRead, &numBytesRead, &errorCode)) {
                
                // Check if this is the motor we're looking for
                if (serialNo == targetSerialNo) {
                    // Found the right motor! Now initialize it
                    VCS_ClearFault(handle, nodeID, &errorCode);
                    VCS_SetDisableState(handle, nodeID, &errorCode);
                    
                    // Mark this port as open and store it for this instance
                    openPorts_.insert(portName);
                    portName_ = portName;
                    
                    cout << "[MaxonMotor] Found motor with serial " << serialNo 
                         << " on port " << portName << endl;
                    return handle;
                }
            }
            
            // Not the right motor, close this device and continue scanning
            VCS_CloseDevice(handle, &errorCode);
        }
        // If handle is nullptr, the device failed to open - skip it
    }
    
    // Motor not found
    cerr << "[MaxonMotor] Could not find motor with serial number " << targetSerialNo << endl;
    return nullptr;
}

// X, Y, Z Maxon Motor ctor
MaxonMotor::MaxonMotor(MaxonParameters& params, Axis axis)
    : Actuator(axis),
      serialNo_(params.serialNo),
      motorName_(params.motorName),
      torqueConstant_(params.KT),
      momentOfInertia_(params.momentOfInertia),
      maxVelocity_(params.maxVelocity),
      handle_(nullptr),
      params(params)
{
    // Open the motor by scanning USB ports for matching serial number
    handle_ = openMotorBySerialNumber(params.serialNo);
    
    if (handle_ != nullptr) {
        // Initialize motor settings (gains, encoder, velocity mode, etc.)
        initSettings(params);
        isOpen_ = true;
        cout << "[MaxonMotor] " << motorName_ << " initialized on axis " 
             << static_cast<int>(axis_) << endl;
    } else {
        cerr << "[MaxonMotor] Failed to open motor " << motorName_ 
             << " with serial number " << serialNo_ << endl;
        isOpen_ = false;
    }
}

// L & R Torp Arm Maxon Motors ctor
MaxonMotor::MaxonMotor(MaxonParameters& params, Side side)
    : Actuator(side),
      serialNo_(params.serialNo),
      motorName_(params.motorName),
      torqueConstant_(params.KT),
      momentOfInertia_(params.momentOfInertia),
      maxVelocity_(params.maxVelocity),
      handle_(nullptr),
      params(params)
{
    handle_ = openMotorBySerialNumber(params.serialNo);

    if (handle_!= nullptr) {
        // Initialize motor settings (gains, encoder, velocity mode, etc.)
        initSettings(params);
        isOpen_ = true;
        cout << "[MaxonMotor] " << motorName_ << "initalized on side " 
             << static_cast<int>(side_) << endl;
    } else {
        cerr << "[MaxonMotor] Failed to open motor " << motorName_
             << " with serial number " << serialNo_ << endl;
             isOpen_ = false;
    }
}

MaxonMotor::~MaxonMotor() {
    if (isOpen_) {
        // Stop the motor
        VCS_HaltVelocityMovement(handle_, nodeID_, &errorCode_);
        
        // Disable the motor
        setDisableState();
        
        // Close the device
        VCS_CloseDevice(handle_, &errorCode_);
        
        // Remove this port from the open ports tracking
        openPorts_.erase(portName_);
        
        cout << "[MaxonMotor] " << motorName_ << " closed and port " 
             << portName_ << " released" << endl;
    }
}

void MaxonMotor::initSettings(const MaxonParameters& params) {
    // Clear any existing faults
    clearFault();
    
    // Switch to disable state
    setDisableState();
    
    // Set protocol stack settings
    if (!VCS_SetProtocolStackSettings(handle_, baudrate_, timeout_, &errorCode_)) {
        cerr << "[MaxonMotor] Failed to set protocol stack settings" << endl;
    }
    
    // Set encoder parameters
    if (!VCS_SetSensorType(handle_, nodeID_, ST_INC_ENCODER_2CHANNEL, &errorCode_)) {
        cerr << "[MaxonMotor] Failed to set sensor type" << endl;
    }
    if (!VCS_SetIncEncoderParameter(handle_, nodeID_, params.NUM_OF_PULSE_PER_REV, params.SENSOR_POLARITY, &errorCode_)) {
        cerr << "[MaxonMotor] Failed to set encoder parameters" << endl;
    }
    
    // Set current controller gains
    if (!VCS_SetControllerGain(handle_, nodeID_, EC_PI_CURRENT_CONTROLLER, EG_PICC_P_GAIN, params.KP_CURR, &errorCode_)) {
        cerr << "[MaxonMotor] Failed to set current P gain" << endl;
    }
    if (!VCS_SetControllerGain(handle_, nodeID_, EC_PI_CURRENT_CONTROLLER, EG_PICC_I_GAIN, params.KI_CURR, &errorCode_)) {
        cerr << "[MaxonMotor] Failed to set current I gain" << endl;
    }
    
    // Set velocity controller gains
    if (!VCS_SetControllerGain(handle_, nodeID_, EC_PI_VELOCITY_CONTROLLER, EG_PIVC_P_GAIN, 
                                static_cast<unsigned long long>(params.K_P * params.K_TUNING), &errorCode_)) {
        cerr << "[MaxonMotor] Failed to set velocity P gain" << endl;
    }
    if (!VCS_SetControllerGain(handle_, nodeID_, EC_PI_VELOCITY_CONTROLLER, EG_PIVC_I_GAIN, 
                                static_cast<unsigned long long>(params.K_I * params.K_TUNING * params.K_TUNING), &errorCode_)) {
        cerr << "[MaxonMotor] Failed to set velocity I gain" << endl;
    }
    if (!VCS_SetControllerGain(handle_, nodeID_, EC_PI_VELOCITY_CONTROLLER, EG_PIVC_FEED_FORWARD_VELOCITY_GAIN, 
                                params.KFF_VEL, &errorCode_)) {
        cerr << "[MaxonMotor] Failed to set velocity feedforward gain" << endl;
    }
    if (!VCS_SetControllerGain(handle_, nodeID_, EC_PI_VELOCITY_CONTROLLER, EG_PIVC_FEED_FORWARD_ACCELERATION_GAIN, 
                                params.KFF_ACC, &errorCode_)) {
        cerr << "[MaxonMotor] Failed to set acceleration feedforward gain" << endl;
    }
    
    // Set operation mode to velocity (momentum wheel control)
    if (!VCS_SetOperationMode(handle_, nodeID_, OMD_VELOCITY_MODE, &errorCode_)) {
        cerr << "[MaxonMotor] Failed to set velocity mode" << endl;
    }
    if (!VCS_ActivateVelocityMode(handle_, nodeID_, &errorCode_)) {
        cerr << "[MaxonMotor] Failed to activate velocity mode" << endl;
    }
    
    // Enable the motor
    setEnableState();
}

void MaxonMotor::clearFault() {
    if (!VCS_ClearFault(handle_, nodeID_, &errorCode_)) {
        cerr << "[MaxonMotor] Failed to clear fault" << endl;
    }
}

void MaxonMotor::setEnableState() {
    if (!VCS_SetEnableState(handle_, nodeID_, &errorCode_)) {
        cerr << "[MaxonMotor] Failed to enable motor" << endl;
    }
}

void MaxonMotor::setDisableState() {
    if (!VCS_SetDisableState(handle_, nodeID_, &errorCode_)) {
        cerr << "[MaxonMotor] Failed to disable motor" << endl;
    }
}

void MaxonMotor::applyTorque(const Vector3d& torqueCmd, double deltaT) {
    if (!isOpen_) return;
    
    // Extract torque for this motor's axis
    double torque = torqueCmd(static_cast<int>(axis_));
    
    // Torque -> acceleration -> velocity integration for momentum wheel
    double accCmd = torque / momentOfInertia_;
    
    // Integrate velocity as double to preserve fractional rpm changes
    // conversion factor: (30 / pi) converts rad/s to rpm
    double velocityIncrement = (accCmd * deltaT) * (30.0 / M_PI);
    currentVelocity_ += velocityIncrement;

    // Clamp to max velocity
    if (currentVelocity_ > maxVelocity_) {
        currentVelocity_ = maxVelocity_;
    } else if (currentVelocity_ < -maxVelocity_) {
        currentVelocity_ = -maxVelocity_;
    }
    
    // Convert to int only when commanding the motor
    int velCmd = static_cast<int>(currentVelocity_);
    
    // Send velocity command to motor
    setVelocityCommand(velCmd);
}

Vector3d MaxonMotor::getTorque() const {
    // Calculate actual torque from current
    if (!isOpen_) return Vector3d::Zero();
    
    int currentMA = 0;
    if (!VCS_GetCurrentIsAveragedEx(handle_, nodeID_, &currentMA, 
                                     const_cast<unsigned int*>(&errorCode_))) {
        return Vector3d::Zero();
    }
    
    // Convert current (mA) to torque (Nm): T = KT * I
    double currentA = currentMA / 1000.0;
    double torque = torqueConstant_ * currentA;
    
    Vector3d torqueVec = Vector3d::Zero();
    torqueVec(static_cast<int>(axis_)) = torque;
    return torqueVec;
}

double MaxonMotor::getVelocity() const {
    if (!isOpen_) return 0.0;
    
    int velocity = 0;
    if (!VCS_GetVelocityIsAveraged(handle_, nodeID_, &velocity, 
                                    const_cast<unsigned int*>(&errorCode_))) {
        return 0.0;
    }
    
    return static_cast<double>(velocity);
}

bool MaxonMotor::setVelocityCommand(int velocityRPM) {
    if (!isOpen_) return false;
    
    if (!VCS_SetVelocityMust(handle_, nodeID_, velocityRPM, &errorCode_)) {
        cerr << "[MaxonMotor] Failed to set velocity command: " << velocityRPM << " rpm" << endl;
        return false;
    }
    
    commandedVelocity_ = velocityRPM;
    return true;
}

// Function to set the torp Maxon Motor speeds
bool MaxonMotor:: setVelocity(int velocityRPM) {

    return setVelocityCommand(velocityRPM);

}

double MaxonMotor::getPosition() const { // Function to get the position of the motor

    if (!isOpen_) return 0.0;

    int position = 0;

    if (!VCS_GetPositionIs(handle_, nodeID_, &position, const_cast<unsigned int*>(&errorCode_))) {

        return 0.0;
    }
    return (double)position * (360.0 / (4.0 * params.NUM_OF_PULSE_PER_REV));

}

// Halt motion of the motor
void MaxonMotor::haltMotion()
{
    if (!VCS_HaltVelocityMovement(handle_, nodeID_, &errorCode_)) {
        cerr << "[MaxonMotor] Failed to halt motor motion." << endl;
    }
}