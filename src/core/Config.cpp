#include "Config.hpp"
#include "core/utils/RotationHelpers.hpp"

std::vector<RotationCommand> AttitudeConfig::getRotationQueue()
{
    return {
        RotationCommand(Vector3d{0.0, 0.0, 1.0}, M_PI / 30.0, vel, acc),
    };
}
