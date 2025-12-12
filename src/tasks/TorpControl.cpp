#include "tasks/TorpControlTask.hpp"
#include "core/utils/TimeUtils.hpp"
#include "core/control/PIControl.hpp"
#include "hardware/sensors/Labjack.hpp"
#include "core/utils/LogHelpers.hpp"
#include "Config.hpp"
#include <iostream>
#include <stdexcept>
#include <cmath>

using namespace std;
using namespace TimeUtils;
// static initialization
bool TorpControl::leftHomingFlag = false;
bool TorpControl::rightHomingFlag = false;
bool TorpControl::enabled = false;

TorpControl::TorpControl(
                TorpMaxonActuator& torpMaxonActuator,
                TorpStepperActuator& torpStepperActuator,
                TorpPIControl& torpPIControl,
                LabJackU6& labJack,
                LJEncoder& encoder,
                Side side)
    : BaseTask("TorpControl", 1),
      torpMaxonActuator_(torpMaxonActuator),
      torpStepperActuator_(torpStepperActuator),
      torpPIControl_(torpPIControl),
      labjack_(labJack),
      encoder_(encoder),
      side_(side),
      leftMotor(torpPIControl.leftMotor),
      rightMotor(torpPIControl.rightMotor)

{

    // PI control handle for picking motor
    pi_ = (side_ == Side::L) ? &leftMotor : &rightMotor;

    // Homing variables
    homingVel    = torpMaxonActuator_.getHomingVel(side_);
    maxAcc       = torpMaxonActuator_.getMaxAcc(side_);
    offsetPos    = torpMaxonActuator_.getOffsetPos(side_);
    offsetPosLim = torpMaxonActuator_.getOffsetPosLim(side_);
    startPosLim  = torpMaxonActuator_.getStartPosLim(side_);
    flipSign = (homingVel >= 0) ? false : true; 
    leftHomingFlag = false;
    rightHomingFlag = false;
    doneHomingFlag = false;
    startPosLocFlag = false;
    torpPrePos = 0.0;
    
    // Torp Master Control variables
    oprVelMag = getSignDir(flipSign) * TorpConfig::oprVel;
    tAccDec = TorpConfig::tAccDec;
    tCruise = TorpConfig::tCruise;
    tDeployRetract = TorpConfig::tDeployRetract;

    maxAccAllowed = abs(2 * oprVelMag / tAccDec);

    deltaTaskTime = TorpConfig::deltaTaskTime;
    nextTaskTime = 0;
    state = INITIALIZING_TORP;
    nextState = INITIALIZING_TORP;

    cout << "[TorpControl] Torp sequence starting." << endl;

    // Initialize logging in separate thread
}

TorpControl::~TorpControl()
{
    cout << "[TorpControl] Shutting down." << endl;
}

int TorpControl::Run()
{
    if (enabled) {

        if (!TorpConfig::controlBodyOnlyFlag) { 

            if (GetTimeNow() < nextTaskTime) {

                return 0;
            }

            // update
            timeStart = GetTimeNow();
            state = nextState;
            stateName = nextStateName;
            time = GetTimeNow();

            motPos = torpMaxonActuator_.getMotPos(side_);
            motVel = torpMaxonActuator_.getSpeed(side_);
            torpPos = getAngularPosDeg();

            // sets preTime on initial run
            if (firstRunFlag) {

                preTime = GetTimeNow();
                firstRunFlag = false;

            }

            deltaT = GetTimeNow() - preTime; // fresh deltaT for going into the switch statement

            // Moving average filter applied to velocity calculation for noise reduction
            torpVel = velMAFilter.addSample(((torpPos - torpPrePos) / 360.0) / (deltaT / 60.0));
            torpPrePos = torpPos; // position stored as pre-position for next cycle
            
            switch(state)
            {
                // State function: calculates Maxon acceleration time, sets reference acceleration
                case INITIALIZING_TORP: 

                    nextState = FINDING_HOME_INDEX;

                    cout << "[TorpControl] 1. Initialization state, setting timing variables." << endl;

                    refPos = torpPos;
                    tA = GetTimeNow(); // start time for the acceleration phase
                    tB = tA + abs(homingVel / maxAcc) - deltaTaskTime; // end time of acceleration phase -- t = v/a
                    refAcc = ((double) getSignDir(flipSign)) * maxAcc; // reference acceleration set to maxAcc

                    cout << "[TorpControl] 2. Finding home index state, locating index position." << endl;

                    break;
                // State function:
                // 1. Retrieves motor positon and velocity from Maxon controllers, torp position from labjack encoder
                // and calculates the torp velocity from torp position over time on every cycle.
                // 2. Accelerates torp arms to homing velocity and searches for index/home position on encoder
                // 3. Determines deceleration profile required for stopping at the starting position
                case FINDING_HOME_INDEX:

                    // Velocity profile control: trapezoidal profile
                    // 1. t < tB, acc phase - integrate acceleration
                    // 2. t >= tB, constant velocity phase
                    if (time > tB) {

                        refAcc = 0;
                        refVel = homingVel;

                    } else {

                        refVel = refVel + refAcc * deltaT;

                    }
                    // Home position/index detection, latching, flag setting
                    if (getIndexFlag()) {
                        
                        cout << "[TorpControl] 2. Finding home index state, index position located." << endl;

                        indexFlag = true;
                        nextState = MOVING_TO_START_POS;
                        homeTorpPos = torpPos;
                        homeMotPos = motPos;

                        // Could rewrite this section using motor encoder logic instead of labjack encoder
                        // Labjack logic was used in previous code --

                        startPosRef = (homeTorpPos + ((double) getSignDir(flipSign)) * offsetPos);

                        // Deceleration profile control for stopping at home position:
                        if (abs(offsetPos) >= offsetPosLim) {

                            // constant deceleration minimum: a = v^2/(2*d), x6 -- safety margin
                            double acc_min = abs((pow(refVel, 2) / (2 * offsetPos)) * 6);

                            if (acc_min >= maxAcc) {

                                accMag = acc_min; // Acceleration cannot exceed max acceleration
                                
                            } else if (acc_min <= 0.2 * maxAcc) {

                                accMag = maxAcc; // Aggressive deceleration if minimum is <= 20% of minimum

                            } else {

                                accMag = 5.0 * acc_min; // Moderate deceleration for all other cases
                            }
                            
                            // tA, tB create a deceleration window:
                            // tA & tB = d/v [time at acc = 0] + current time +/- v/(2*acc) [half the deceleration time]
                            tA = abs((offsetPos / CONV_RAD_TO_DEG) / (refVel * CONV_RPM_TO_RADpSEC)) + GetTimeNow() - abs(refVel / (2 * accMag));
                            tB = abs((offsetPos / CONV_RAD_TO_DEG) / (refVel * CONV_RPM_TO_RADpSEC)) + GetTimeNow() + abs(refVel / (2 * accMag)); 
                        
                        } else { 

                            // offset is small enough, no deceleration needed
                            accMag = 0;
                            tA = GetTimeNow();
                            tB = tA;

                        }
                    }

                    break;
                
                // State function:
                // 1. Retrieves motor positon and velocity from Maxon controllers, torp position from labjack encoder,
                // calculates the torp velocity from torp position and calculates over time on every cycle.
                // Move torp arms to starting position, offset from home index
                case MOVING_TO_START_POS:
                    
    
                    // Deceleration control
                    // Only decelerate during deceleration window [tA, tB]
                    if (time >= tA && time <= tB) {

                        refAcc = ((double) getSignDir(flipSign)) * -accMag;

                    } else {

                        refAcc = 0; // no deceleration

                    }
                    
                    // After deceleration window, commanded velocity = 0
                    if (time > tB) {
                        refVel = 0;

                    } else {

                        // Before window ends, integrate acceleration to update velocity
                        refVel = refVel + (refAcc * deltaT); 
                    }

                    // Could rewrite this section using motor encoder logic instead of labjack encoder
                    // Labjack logic was used in previous code --

                    // Start location positioning:
                    // Outer condition: sets startPosLocFlag when position within 125% of limit or small velocity
                    if (abs(torpPos - startPosRef) <= 1.25 * startPosLim || abs(refVel) <=0.035) {
                        cout << "[Torp Control] Torp starting position found" << endl;
                        startPosLocFlag = true;

                        // Inner condition: sets startPosAct when position error inside limit, commanded 
                        // velocity stops, and torp arms are moving slowly or stopped -- stops motors, sets flags,
                        // changes state
                        if ((abs(torpPos - startPosRef) <= startPosLim) && (abs(refVel) <= 0.01) && (abs(torpVel) <= 0.25)) {

                            startPosAct = torpPos;
                            torpMaxonActuator_.haltMotion(side_);

                            if (side_ == Side::L) {

                                leftHomingFlag = true;

                            } else {

                                rightHomingFlag = true;
                            }

                        }
                    }

                    // When both arms finished homing, transition state
                    if (rightHomingFlag && leftHomingFlag) {
                
                        doneHomingFlag = true;
                        // Initialize S-curve accumulators NOW so end-of-tick integration is safe
                        posProfVal = startPosAct;
                        velProfVal = 0.0;
                        accProfVal = 0.0;
                        jerkProfVal = 0.0; // will be set in WAITING_FOR_SYNC on the next tick

                        nextState = WAITING_FOR_SYNC;
                    }

                    break;

                // State function:
                // Check for both arms to finish homing process, set values for S-Curve acceleration profile
                case WAITING_FOR_SYNC:
                    
                    if (maxAcc >= maxAccAllowed) {
                        maxAcc = maxAccAllowed; // Clamp acceleration to maximum allowed
                        }

                    // S-Curve acceleration profile initialization
                    refAcc = ((double) getSignDir(flipSign)) * maxAcc; // reference acceleration set to maxAcc

                    T2 = abs(2 * oprVelMag / refAcc) - tAccDec; // constant acceleration phase duration
                    T1 = 0.5 * (tAccDec - T2); // ramp-up & ramp-down time

                    Ta = time + T1; // Time at end of ramp-up, beginning of constant acceleration
                    Tb = time + T1 + T2; // End of constant acceleration, time at start of ramp-down
                    Tc = time + tAccDec; // Total time of acceleration
                    
                    posProfVal = startPosAct;
                    velProfVal = 0.0;
                    accProfVal = 0.0;
                    jerkProfVal = refAcc / T1; // Ramp-up acceleration rate of change profile
                    cout<< "[TorpControl] Starting to accelerate with jerk: " << jerkProfVal << endl
                        << "Ta: " << Ta << endl
                        << "Tb: " << Tb << endl
                        << "Tc: " << Tc << endl;

                         

                    nextState = ACCELERATING;

                    break;

                    
                // State function: 
                // Execute S-curve acceleration profile to reach operating velocity
                case ACCELERATING:

                    // Acceleration is finished
                    if (time >= Tc) {
                        cout << "[Torp Control] Done accelerating, going into cruising." << endl
                             << "Ref vel: " << refVel << endl    
                             << "Opr vel: " << oprVelMag << endl;
                        nextState = CRUISING;
                        readyToDeployFlag = true;
                        deployStartFlag = true;

                        Td = time + tCruise; // Time at end of cruising
                        velProfVal = oprVelMag; // Constant velocity after acceleration

                        // Acceleration ended
                        accProfVal = 0; 
                        jerkProfVal = 0;

                    } else {
                        // Jerk profile implementation -- already running condition (time < Ta)
                        if (time >= Ta && time <= Tb) {

                            jerkProfVal = 0; // Jerk = 0 during constant acceleration phase

                        } else if (time > Tb) {

                            jerkProfVal = -1 * refAcc / T1; // Ramp-down acceleration rate of change profile
                        }
                    }   

                    break;

                case CRUISING:
                    
                    if (time >= Td) {

                        // Masses are deploying, not finished
                        if (deployStartFlag && !deployDoneFlag) {
                            nextState = DEPLOYING_MASS;
                            tEndDeployRetract = time + tDeployRetract;
                        
                        // Signal to spin down from task coordinator
                        } else if (deployDoneFlag && startSpinningDownFlag) { 

                            retractStartFlag = true;
                        
                        // Masses are retracting, not finished
                        } else if (retractStartFlag && !retractDoneFlag) {

                            nextState = RETRACTING_MASS;
                            tEndDeployRetract = time + tDeployRetract;

                        // Masses have finished retracting, cruising finished
                        } else if (deployDoneFlag && retractDoneFlag) {

                            nextState = DECELERATING;

                            // Calculate deceleration parameters
                            Ta = time + T1;
                            Tb = time + T1 + T2;
                            Tc = time + tAccDec;
                            jerkProfVal = -1 * refAcc / T1;

                        // Cruising between deployment and retraction
                        } else {

                            velProfVal = oprVelMag;
                            accProfVal = 0;
                            jerkProfVal = 0;
                        }
                    }

                    break;
                
                case DEPLOYING_MASS:
                    
                    if (!deployDoneFlag && !deployCommandSent) { // deployment not finished 
                        double distPerStep = 0.04; // linear distance per step (mm)
                        double lBoom = 200; // travel distance of each boom
                        int32_t deployTargetPos = (int32_t)(lBoom / distPerStep);
                        // Run steppers to desired position
                        torpStepperActuator_.runToPositionSide(side_, deployTargetPos);
                        deployCommandSent = true;
                        cout << "[Torp Control] Deploy command sent for masses" << endl;
                    }

                    // Continue to run the Maxons at cruising speed
                    velProfVal = oprVelMag;
                    accProfVal = 0;
                    jerkProfVal = 0;
                    if (time >= tEndDeployRetract) { // Deployment window has finished

                        nextState = CRUISING;
                        Td = time + tCruise; // Post-deployment cruising time set
                        deployDoneFlag = true; // Deployment has finished
                    }

                    break;

                case RETRACTING_MASS:

                    if (!retractDoneFlag && !retractCommandSent) { // retraction not finished

                        double distPerStep = 0.04;
                        double lBoom = 200;
                        int32_t retractTargetPos = -(int32_t)((lBoom - 10) / distPerStep);

                        // Run steppers to desired position
                        torpStepperActuator_.runToPositionSide(side_, retractTargetPos);
                        retractCommandSent = true;
                        cout << "[Torp Control] Retract command sent for masses" << endl;

                        
                    }

                    if (time >= tEndDeployRetract) { // Retraction window has finished

                        // De-energize stepper motors after retraction
                        torpStepperActuator_.DeEnergizeSide(side_);

                        nextState = CRUISING;
                        Td = time + tCruise; // Post-retraction cruise time set
                        retractDoneFlag = true; // Retraction has finished
                    }

                    break;

                case DECELERATING:
                    
                    if (maxAcc >= maxAccAllowed) {
                        maxAcc = maxAccAllowed; // Clamp acceleration to maximum allowed
                    }

                    if (time >= Tc) { // Deceleration window ended

                        nextState = STOPPING;
                        velProfVal = 0;
                        accProfVal = 0;
                        jerkProfVal = 0;

                    // Enters deceleration state in (time < Ta) case, initial jerkProfVal set in Cruising
                    } else {

                        // 
                        if (time >= Ta && time <= Tb) { // Constant deceleration window

                            jerkProfVal = 0; 

                        } else if (time > Tb) { // Deceleration ramp-down window

                            jerkProfVal = abs(maxAcc / T1); 
                        }
                    }
                
                case STOPPING:
                    
                    torpMaxonActuator_.haltMotion(side_);
                    cout << "[Torp Control] Torp actuation phase completed. Torp arms stopped." << endl;

                    break;

            }
            
            if (doneHomingFlag) { // S-curve acceleration profile calculation

                // Integrate acceleration, velocity, and position profiles
                accProfVal = accProfVal + jerkProfVal * deltaT;
                velProfVal = velProfVal + accProfVal * deltaT;
                posProfVal = posProfVal + (velProfVal * (deltaT / 60.0) * 360.0);

                refVel = velProfVal;
                refPos = posProfVal;


            } else { // Homing/Start logic
                if (startPosLocFlag) {
                    // At start position
                    refVel = 0.0;
                    refPos = startPosRef;

                } else {

                    // Homing
                    refPos = refPos + refVel * (deltaT / 60.0) * (360.0);
                }
            }


            double piSignal = pi_->calculate(refPos, torpPos);
            
            // TODO Figure out why we are getting -nan for piSignal when refPos and torpPos are positive during the acclerating phase
            desVel = piSignal / 360 + refVel;
            posErr = pi_->getError();
        
            int signal = roundingFunc(desVel * TorpConfig::gearRatio);
            // Actuate torp Maxon Motors
            if (!torpMaxonActuator_.setVelocity(side_, signal)) {
                cout << "[Torp Control] Failed to set torp velocity" << endl;
            }
        }

        else {
            deployDoneFlag = true;  // Notify its done
        }

    }
    preTime = time;

    timeEnd = GetTimeNow();
    nextTaskTime += deltaTaskTime;
    
    return 0;

}


// Helper functions:

int TorpControl::getSignDir(bool flipSignFlag) {
         
    int signResult = (!flipSignFlag) ? 1 : -1;
    return signResult;

    }


int TorpControl::roundingFunc(double val) {

    return static_cast<int>(std::lround(val));

}