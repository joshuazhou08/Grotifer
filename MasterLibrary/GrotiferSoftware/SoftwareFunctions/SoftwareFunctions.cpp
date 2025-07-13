#include <unistd.h>
#include <iostream>
#include <string>
#include <string.h>
#include <time.h>
#include <fstream>
#include <iomanip>
#include <math.h>
#include <cmath>
#include <fcntl.h>
#include <termios.h>
#include <stdio.h>
#include <errno.h>
#include <signal.h>
#include <stdlib.h>
#include <chrono>

#include "SoftwareFunctions.hpp"

using namespace std;

// Converting functions
double Deg2Rad(double angleVal) // Function to convert from deg to radian
{
    return angleVal * M_PI / 180.0;
}

double Rad2Deg(double angleVal) // Function to convert from rad to deg
{
    return angleVal * 180.0 / M_PI;
}

// Function to get elapsed time from the initial time t0 [s]
double GetTimeNow()
{
    using clock = std::chrono::steady_clock;
    static const auto t0 = clock::now(); // one-time zero point
    auto now = clock::now();
    std::chrono::duration<double> elapsed = now - t0;
    return elapsed.count(); // seconds as double
}

// PI Position control
PIControl::PIControl(PIControlPara &pic)
{
    p_kp = pic.kp;
    p_ki = pic.ki;
    p_hLim = pic.hLim;
    p_lLim = pic.lLim;
};

PIControl::PIControl(double kp, double ki, double hLim, double lLim, double kd)
    : p_kp(kp), p_ki(ki), p_hLim(hLim), p_lLim(lLim), p_kd(kd) {
      };

PIControl::~PIControl() {};

// Get data function
double PIControl::GetKp() { return p_kp; }       // Get k_p
double PIControl::GetKi() { return p_ki; }       // Get k_i
double PIControl::GetError() { return p_error; } // Get the error from the PI controller

// Main Controller function
double PIControl::PIDCalculation(double setpoint, double actVal)
{
    p_setpoint = setpoint;
    p_actVal = actVal;

    p_time = GetTimeNow();         // Get the time when starting to calculate the control signal
    p_deltaT = p_time - p_preTime; // Calculate the actual time it take between two calls

    if (p_deltaT > 1) // If deltaT is called 1 second apart it means p_preTime is invalid (first run)
    {
        p_deltaT = 0;
    }
    p_preTime = p_time; // Assign current time into previous time

    p_error = p_setpoint - p_actVal; // Calculate error

    // Calculate each part of the controller
    p_P_part = p_kp * p_error;

    // Calculate P-part of the controller
    p_P_part = p_kp * p_error;
    // Calculate the I-part with anti-windup
    if ((p_P_part + p_I_part > p_hLim) && (p_error > 0))
        p_I_part = p_I_part;
    else if ((p_P_part + p_I_part < p_hLim) && (p_error < 0))
        p_I_part = p_I_part;
    else
        p_I_part += p_ki * p_error * p_deltaT;

    // D part
    if (p_deltaT > 0.0)
    {
        const double derivative = (p_error - p_prevError) / p_deltaT;
        p_D_part = p_kd * derivative;
    }
    else
    {
        p_D_part = 0.0;
    }

    double u = p_P_part + p_I_part + p_D_part;
    if (u > p_hLim)
        u = p_hLim;
    else if (u < p_lLim)
        u = p_lLim;

    // 7) Save for next time
    p_prevError = p_error;
    p_uPID = u;

    return p_uPID;
}
