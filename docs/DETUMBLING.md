# Detumbling Control Algorithm

## Overview

The detumbling control algorithm is designed to reduce the angular velocity of the spacecraft to near-zero levels. This is a critical phase that occurs after deployment or when the spacecraft needs to stabilize its rotation before transitioning to attitude control.

## Control Algorithm

### Simple Proportional Control

The detumbling algorithm uses a straightforward proportional control approach:

1. **Measure Angular Velocity**: Get the current angular velocity vector `angularVelocityVec`
2. **Apply Proportional Control**: Multiply each component by a proportional gain constant
3. **Negate the Result**: Apply negative feedback to oppose the current rotation
4. **Apply Torque**: Send the calculated torque to the reaction wheels

### Mathematical Implementation

```cpp
Vector3d torque;
torque(0) = -AttitudeConfig::xVelocityK_p * angularVelocityVec(0);
torque(1) = -AttitudeConfig::yVelocityK_p * angularVelocityVec(1);
torque(2) = -AttitudeConfig::zVelocityK_p * angularVelocityVec(2);

applyTorque(torque, deltaT);
```

### Control Parameters

- **xVelocityK_p**: Proportional gain for X-axis angular velocity
- **yVelocityK_p**: Proportional gain for Y-axis angular velocity
- **zVelocityK_p**: Proportional gain for Z-axis angular velocity

## Termination Conditions

The detumbling phase is considered complete when both conditions are met:

1. **Time Elapsed**: `time >= detumblingEndTime`
2. **Angular Velocity Threshold**: Maximum angular velocity component is below `4.5e-3` rad/s

```cpp
double max_component = angularVelocityVec.cwiseAbs().maxCoeff();

if (time >= detumblingEndTime && max_component < 4.5e-3)
{
    detumblingDone = true;
    logger.info("Detumbling Done");
}
```

## State Machine Integration

The detumbling algorithm operates in the `DETUMBLING` state:

- **State Entry**: Initialize detumbling parameters and timing
- **State Execution**: Apply proportional control to reduce angular velocity
- **State Exit**: Transition to `DETERMINING_ATTITUDE` when detumbling is complete

## Key Characteristics

### Simplicity

- Uses only proportional control (no integral or derivative terms)
- Direct negative feedback approach
- Minimal computational complexity

### Effectiveness

- Rapidly reduces angular velocity to acceptable levels
- Provides smooth deceleration without overshoot
- Suitable for post-deployment stabilization

### Robustness

- Works regardless of initial angular velocity magnitude
- Handles asymmetric rotation rates across axes
- Tolerant to measurement noise and disturbances

## Advantages

1. **Simple Implementation**: Easy to understand and debug
2. **Fast Response**: Direct proportional control provides immediate response
3. **Stable Performance**: Negative feedback ensures convergence
4. **Resource Efficient**: Minimal computational overhead
5. **Reliable**: Proven approach for spacecraft detumbling

## Usage Example

```cpp
case DETUMBLING:
{
    double time = GetTimeNow();
    double deltaT = time - preTimeDetumbling;

    // Calculate control torque
    Vector3d torque;
    torque(0) = -AttitudeConfig::xVelocityK_p * angularVelocityVec(0);
    torque(1) = -AttitudeConfig::yVelocityK_p * angularVelocityVec(1);
    torque(2) = -AttitudeConfig::zVelocityK_p * angularVelocityVec(2);

    // Apply the torque
    applyTorque(torque, deltaT);

    // Check termination conditions
    double max_component = angularVelocityVec.cwiseAbs().maxCoeff();
    if (time >= detumblingEndTime && max_component < 4.5e-3)
    {
        detumblingDone = true;
        logger.info("Detumbling Done");
    }

    nextState = DETERMINING_ATTITUDE;
    preTimeDetumbling = time;
    break;
}
```

This algorithm provides a reliable and efficient method for stabilizing spacecraft rotation before transitioning to more sophisticated attitude control systems.
