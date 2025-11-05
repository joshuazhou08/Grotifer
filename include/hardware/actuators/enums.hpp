#pragma once

// Common enums shared across hardware components

// Safer scoped enum (prevents name collisions)
enum class Axis { X = 0, Y = 1, Z = 2, Unused = 99 };

// Not used yet but, example of what this file is used for
enum class ActuatorType { Fan, Wheel };

// Enum for Torp Maxon Motors
enum class Side { L = 0, R = 1, Unused = 99 };

//Enum for Stepper Motors
enum class StepperMot { L1 = 0, L2 = 1, R1 = 2, R2 = 3 };

enum class ActuatorParam { Axis, Side }; 