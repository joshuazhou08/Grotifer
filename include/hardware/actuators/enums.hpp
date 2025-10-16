#pragma once

// Common enums shared across hardware components

// Safer scoped enum (prevents name collisions)
enum class Axis { X = 0, Y = 1, Z = 2 };

// Not used yet but, example of what this file is used for
enum class ActuatorType { Fan, Wheel };