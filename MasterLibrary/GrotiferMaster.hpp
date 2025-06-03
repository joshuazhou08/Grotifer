// Include all the header files
#pragma once
#include <math.h>
#include "AttitudeDetermination.hpp"
#include "SoftwareFunctions.hpp"
#include "MovingAverage.h"
#include "Sensors.hpp"
#include "Actuators.hpp"
#include "GrotiferHardware/Definitions.h"

// Definitions of constants
const double PI = M_PI;
const double CONV_TURN2DEG = 360;           // convert from turn to deg
const double CONV_RPM_TO_RADpSEC = PI / 30; // Convert from rpm to rad/s
const double CONV_RAD_TO_DEG = 180 / PI;    // convert from rad to deg
