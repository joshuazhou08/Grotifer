#include <iostream>
#include <string>
#include <signal.h>
#include <atomic>
#include <thread>
#include <chrono>

#include "hardware/actuators/StepperMotor.hpp"
#include "Factory.hpp"
#include "Config.hpp" 

using namespace std;

static std::atomic<bool> g_shouldExit(false);
static StepperMotor* g_motorPtr = nullptr;

void handleSignal(int signum)
{
    std::cerr << "\n[Signal] Caught signal " << signum
              << " — de-energizing motors...\n";

    if (g_motorPtr != nullptr) {
        g_motorPtr->DeEnergizeMotors();
        std::cerr << "[Signal] Motors de-energized.\n";
    }

    g_shouldExit = true;
}

static void usage()
{
    std::cerr << "Usage: retractMass <L1|L2|R1|R2>\n";
}

int main(int argc, char** argv)
{
    // ----------------------------------------------------------
    // Validate command-line input
    // ----------------------------------------------------------
    if (argc != 2) {
        usage();
        return 1;
    }

    std::string motorName = argv[1];
    StepperMot motorId;
    StepperParameters params;

    if (motorName == "L1") {
        motorId = StepperMot::L1;
        params = makeL1StepperParams();
    }
    else if (motorName == "L2") {
        motorId = StepperMot::L2;
        params = makeL2StepperParams();
    }
    else if (motorName == "R1") {
        motorId = StepperMot::R1;
        params = makeR1StepperParams();
    }
    else if (motorName == "R2") {
        motorId = StepperMot::R2;
        params = makeR2StepperParams();
    }
    else {
        std::cerr << "[Error] Invalid motor name: " << motorName << "\n";
        usage();
        return 1;
    }

    // ----------------------------------------------------------
    // Install signal handlers
    // ----------------------------------------------------------
    signal(SIGINT, handleSignal);
    signal(SIGTERM, handleSignal);

    try {
        // ----------------------------------------------------------
        // Initialize motor
        // ----------------------------------------------------------
        StepperMotor motor(params, motorId);
        g_motorPtr = &motor;

        if (!motor.isOpen()) {
            std::cerr << "[Error] Could not open motor " << motorName << "\n";
            return -1;
        }

        std::cout << "[Main] Motor " << motorName << " initialized.\n";

        // ----------------------------------------------------------
        // Command retract movement
        // ----------------------------------------------------------
        int32_t targetPos = -5000;
        std::cout << "[Main] Sending command: targetPos = "
                  << targetPos << "\n";

        motor.runToPosition(targetPos);

        // ----------------------------------------------------------
        // Wait up to 20 seconds (early exit on SIGINT/SIGTERM)
        // ----------------------------------------------------------
        for (int i = 0; i < 20 && !g_shouldExit; i++) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }

        if (g_shouldExit) return 0;

        // ----------------------------------------------------------
        // Safety: always de-energize at end
        // ----------------------------------------------------------
        motor.DeEnergizeMotors();
        std::cout << "[Main] Done. Motor de-energized.\n";

    } catch (const std::exception& e) {
        std::cerr << "[Exception] " << e.what() << "\n";
    }

    return 0;
}
