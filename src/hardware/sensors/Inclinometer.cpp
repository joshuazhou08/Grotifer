#include "hardware/sensors/Inclinometer.hpp"
#include <iostream>

using namespace std;

// Constructor - connects to LabJack and sets analog input channels for X and Y axes
Inclinometer::Inclinometer(LabJackU6& lju6, long xChannel, long yChannel)
    : lju6_(&lju6),
      xChannel_(xChannel),
      yChannel_(yChannel),
      xAngle_(0.0),
      yAngle_(0.0)
{
    cout << "[Inclinometer] Initialized with X channel " << xChannel_ 
         << " and Y channel " << yChannel_ << endl;
}

Inclinometer::~Inclinometer() {
    // Cleanup if needed
}

// Get X-axis angle in degrees (-25° to +25° range)
// Uses Tatsu's calibration: angleX = 12.5317 * voltageX - 30.4959
double Inclinometer::getAngleX() {
    double voltageX = lju6_->getAveVoltageAtChannel(xChannel_);
    xAngle_ = 12.5317 * voltageX - 30.4959;
    return xAngle_;
}

// Get Y-axis angle in degrees (-25° to +25° range)
// Uses Tatsu's calibration: angleY = 12.4982 * voltageY - 29.4607
double Inclinometer::getAngleY() {
    double voltageY = lju6_->getAveVoltageAtChannel(yChannel_);
    yAngle_ = 12.4982 * voltageY - 29.4607;
    return yAngle_;
}

