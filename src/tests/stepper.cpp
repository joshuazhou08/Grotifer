// Deploys the masses and retracts them with the steppers
#include <iostream>
#include "hardware/actuators/StepperMotor.hpp"
#include "Factory.hpp"
#include "Config.hpp"
#include <thread>
#include <signal.h>
#include <atomic>
#include <chrono>

using namespace std;


std::atomic<bool> g_shouldExit(false);
StepperMotor* g_motorPtr = nullptr;

void handleSignal(int signum)
{
    std::cerr << "\n[Signal] Caught signal " << signum << ", de-energizing motors...\n";

    if (g_motorPtr != nullptr) {
        g_motorPtr->DeEnergizeMotors();
        std::cerr << "[Signal] Motors de-energized.\n";
    }

    g_shouldExit = true;
}

int main() {

    // Install Ctrl-C and kill handlers
    signal(SIGINT, handleSignal);
    signal(SIGTERM, handleSignal);

    try {
        StepperParameters testParams = makeL1StepperParams();
        StepperMotor TestMotor(testParams, StepperMot::L1);

        // Save pointer so signals can de-energize
        g_motorPtr = &TestMotor;

        if (!TestMotor.isOpen()) {
            std::cerr << "[Main] Failed to initialize L1 Stepper Motor\n";
            return -1;
        }

        std::cout << "[Main] Stepper Motor initialized\n";

        int32_t deployTargetPos = 1000;

        TestMotor.runToPosition(deployTargetPos);

        // Safe sleep (exit early on signal)
        for (int i = 0; i < 15 && !g_shouldExit; i++)
            std::this_thread::sleep_for(std::chrono::seconds(1));

        if (g_shouldExit) return 0;

        int32_t retractTargetPos = -1000;
        TestMotor.runToPosition(retractTargetPos);

        for (int i = 0; i < 15 && !g_shouldExit; i++)
            std::this_thread::sleep_for(std::chrono::seconds(1));

        if (g_shouldExit) return 0;

        // Optional: turn off motors at end
        TestMotor.DeEnergizeMotors();

    } catch (const std::exception& e) {
        std::cerr << "Stepper test failed: " << e.what() << std::endl;
    }
}
