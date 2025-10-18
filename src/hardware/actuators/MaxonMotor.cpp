#include "hardware/actuators/MaxonMotor.hpp"
#include <Definitions.h>
#include <iostream>
#include <cmath>
#include <cstring>

using namespace std;
using namespace Eigen;

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
    
    // Scan for available ports
    const unsigned int MAX_STR_SIZE = 100;
    char portNameTemp[MAX_STR_SIZE];  // Temporary buffer to receive each port name during scan
    int endOfSelection = false;
    
    // Get first port
    if (!VCS_GetPortNameSelection(deviceName, protocolStackName, interfaceName, true, 
                                   portNameTemp, MAX_STR_SIZE, &endOfSelection, &errorCode)) {
        cerr << "[MaxonMotor] Failed to scan for USB ports" << endl;
        return nullptr;
    }
    
    // Try each available port until we find our motor
    while (!endOfSelection) {
        // Try to open this port
        void* handle = VCS_OpenDevice(deviceName, protocolStackName, interfaceName, 
                                      portNameTemp, &errorCode);
        
        if (handle != nullptr) {
            // Clear any faults and disable
            VCS_ClearFault(handle, nodeID, &errorCode);
            VCS_SetDisableState(handle, nodeID, &errorCode);
            
            // Read serial number from this device
            uint32_t serialNo = 0;
            unsigned int numBytesToRead = 4;
            unsigned int numBytesRead = 0;
            
            if (VCS_GetObject(handle, nodeID, 0x1018, 0x04, &serialNo, 
                             numBytesToRead, &numBytesRead, &errorCode)) {
                
                // Check if this is the motor we're looking for
                if (serialNo == targetSerialNo) {
                    cout << "[MaxonMotor] Found motor with serial " << serialNo 
                         << " on port " << portNameTemp << endl;
                    return handle;
                }
            }
            
            // Not the right motor, close this device and continue scanning
            VCS_CloseDevice(handle, &errorCode);
        }
        
        // Get next port
        VCS_GetPortNameSelection(deviceName, protocolStackName, interfaceName, false,
                                portNameTemp, MAX_STR_SIZE, &endOfSelection, &errorCode);
    }
    
    // Motor not found
    cerr << "[MaxonMotor] Could not find motor with serial number " << targetSerialNo << endl;
    return nullptr;
}

MaxonMotor::MaxonMotor(MaxonParameters& params, Axis axis)
    : Actuator(axis),
      serialNo_(params.serialNo),
      motorName_(params.motorName),
      torqueConstant_(params.KT),
      momentOfInertia_(params.momentOfInertia),
      maxVelocity_(params.maxVelocity),
      handle_(nullptr)
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

MaxonMotor::~MaxonMotor() {
    if (isOpen_) {
        // Stop the motor
        VCS_HaltVelocityMovement(handle_, nodeID_, &errorCode_);
        
        // Disable the motor
        setDisableState();
        
        // Close the device
        VCS_CloseDevice(handle_, &errorCode_);
        
        cout << "[MaxonMotor] " << motorName_ << " closed" << endl;
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
    
    // Get current velocity
    currentVelocity_ = static_cast<int>(getVelocity());
    
    // Integrate: velocity = current + (acceleration * time) * conversion_factor
    // conversion factor: (30 / pi) converts rad/s to rpm
    int velCmd = currentVelocity_ + static_cast<int>((accCmd * deltaT) * (30.0 / M_PI));
    
    // Clamp to max velocity
    if (velCmd > maxVelocity_) {
        velCmd = maxVelocity_;
    } else if (velCmd < -maxVelocity_) {
        velCmd = -maxVelocity_;
    }
    
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

