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

using namespace std;

// Struct-type variables
struct PIControlPara // Struct-type variable to store parameters for PI control
{
    double kp, // P Gain Coefficient
        ki,    // I Gain coefficient
        hLim,  // High Limit for anti wind up
        lLim;  // Low Limit for anti wind up
};

struct homingProfilePara // Struct type variable to store parameters for homing profile
{
    double homingVel;
    double maxAcc;
    double offsetPos;
    double offsetPosLim;
    double startPosLim;
};

// Function to convert from deg -> rad
double Deg2Rad(double angleVal);

// Function to convert from rad -> deg
double Rad2Deg(double angleVal);

double GetTimeNow(); // Function to get elapsed time from the initial time t0, [s]

// PI Position Control
class PIControl
{
public:
    PIControl(PIControlPara &pic); // Constructor
    ~PIControl();

    // Get Data functions
    double GetKp();    // Get k_p
    double GetKi();    // Get k_i
    double GetError(); // Get the error from the PI controller

    // Main Controller Code Function
    double PICalculation(double setpoint, double actVal);

protected:
    double p_deltaT = 0;               // Discrete time to calculate the integral, [sec]
    double p_time = 0;                 // Time when the PI Controller is called, [sec]
    double p_preTime = 0;              // Previous time instance when the PI Controller was called, [sec]
    double p_setpoint = 0;             // Setpoint/reference of the controller, [deg]
    double p_actVal = 0;               // Actual value -> actual position
    double p_kp, p_ki;                 // Controller Gains
    double p_P_part = 0, p_I_part = 0; // P_part and I_part of the controller
    double p_uPI = 0;                  // Control signal
    double p_error = 0;                // Error
    double p_hLim, p_lLim;             // Limit of the controller
};
