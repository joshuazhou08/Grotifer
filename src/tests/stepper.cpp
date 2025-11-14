// Deploys the masses and retracts them with the steppers
#include <iostream>
#include "hardware/actuators/StepperMotor.hpp"
#include "Config.hpp"
#include <thread>
#include <chrono>

using namespace std;
// Left Stepper Motor 1 & 2 configuration
StepperParameters makeL1StepperParams() {
    StepperParameters params;
    params.devicePath = "/dev/left_boom_stepper_1";
    params.baudrate = 9600;
    params.stepMode = 0;
    params.maxSpeed = TorpConfig::stepperSpeed;
    params.startSpeed = TorpConfig::stepperSpeed;
    params.maxCurr = 3500;
    return params;
}

int main() {
    try {
        // Configure stepper parameters
        
       
        StepperParameters StepperL1Params = makeL1StepperParams();
        StepperMotor L1Motor(StepperL1Params, StepperMot::L1);

        if (!L1Motor.isOpen()) {
            cerr << "[Main] Failed to initialize L1 Stepper Motor" << endl;
            return -1;
        }
        cout << "[Main] L1 Stepper Motor initialized" << endl;

        double distPerStep = 0.04; // linear distance per step (mm)
        double lBoom = 200; // travel distance of each boom
        int32_t deployTargetPos = (int32_t)(lBoom / distPerStep);

        cout << "Steppers should be running" << endl;
        L1Motor.runToPosition(deployTargetPos);
        std::this_thread::sleep_for(std::chrono::seconds(60));          // 1 minute
        L1Motor.DeEnergizeMotors();
        

    } catch (const std::exception& e) {
        std::cerr << "Stepper test failed: " << e.what() << std::endl;
    }
}
