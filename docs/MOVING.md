# Moving Profile Calculation

## Overview

The moving profile system controls the attitude of the spacecraft by calculating a smooth trajectory from the current position to a target position. The system uses a combination of velocity control and position error correction to achieve precise attitude control.

## Key Concepts

### Relative Angles

All profile angles are calculated **relative to the starting position**. This means:

- The starting orientation serves as the reference frame (0° position)
- Target angles are expressed as angular displacements from this starting position
- The "target position" is represented by `deltaTheta` - the total angular displacement needed

### Target Position (deltaTheta)

The `deltaTheta` variable represents the total angular displacement required to reach the target orientation:

- Calculated as the angle between the current rotation matrix and the desired rotation matrix
- Serves as the primary target for the moving profile
- Updated dynamically based on the current mission state (e.g., sun-finding mode)

## Control System Architecture

### PI Loop Implementation

The system uses a **Proportional-Integral (PI) Loop** for attitude control:

1. **Velocity Loop**: Uses `movingProfileVelocity` to control the rate of rotation
2. **Position Loop**: Compares current position to target position and generates error correction

### Signal Composition

The control signal combines two components:

1. **Velocity Component** (`movingProfileVelocity`):

   - Controls the rate of rotation
   - Provides smooth motion control
   - Prevents overshooting and oscillation

2. **Error Vector Component**:
   - Represents the difference between where we are and where we should be
   - Calculated by comparing rotated starting position matrix to current matrix
   - Provides position correction

## Error Vector Calculation

The error vector is calculated using the following process:

1. **Starting Matrix**: The initial orientation matrix when the movement began
2. **Target Matrix**: The starting matrix rotated by the moving profile angle (where you should be)
3. **Current Matrix**: The actual current orientation matrix (where you actually are)
4. **Error Calculation**: Calculate the rotation matrix that transforms from target matrix to current matrix

### Mathematical Process

```
Error Vector = Rotation Matrix (Target Matrix → Current Matrix)
Error Vector = Rotation Matrix ((Starting Matrix × Rotation Profile) → Current Matrix)
```

This error vector represents the angular displacement needed to reach the target position and is used by the PI controller to generate correction signals.

## Implementation Details

### State Machine Integration

The moving profile is primarily used in the `MOVING` state of the attitude control system:

- **State Entry**: Initialize starting matrix and calculate `deltaTheta`
- **State Execution**: Apply PI control using velocity and error components
- **State Exit**: Transition to `HOLDING_POSITION` when target is reached

### Profile Generation

The system generates a smooth angular profile that:

- Starts from the current position
- Accelerates to the desired velocity
- Maintains constant velocity during the movement
- Decelerates to reach the target position precisely

## Advantages of This Approach

1. **Relative Positioning**: Eliminates cumulative error by using relative angles
2. **Smooth Motion**: Velocity control prevents jerky movements
3. **Precise Control**: Error vector provides continuous position correction
4. **Robust Performance**: PI loop handles disturbances and modeling errors
5. **Adaptive**: Can handle dynamic targets and changing mission requirements

## Usage Example

```cpp
// Calculate target displacement
auto [rotAxis, rotAngle] = calculateRotationAxisAndAngle(solRotMatBackward, Matrix3d::Identity());
deltaTheta = rotAngle;
movingProfileRotAxis = rotAxis;

// PI control combines velocity and error components
Vector3d signal = velocityComponent + errorComponent;
```

This system provides precise, smooth attitude control while maintaining robustness against external disturbances and modeling uncertainties.

NOTE: calculateRotationAxisAndAngle currently flips the x component of the rotaxis to match our system. This should be investigated further.
