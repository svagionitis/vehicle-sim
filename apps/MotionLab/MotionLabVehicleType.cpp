#include "apps/MotionLab/MotionLabVehicleType.h"

namespace motion_lab {

std::string_view getVehicleTypeName(MotionLabVehicleType type) noexcept
{
    switch (type) {
    case MotionLabVehicleType::ClassicCar:
        return "CLASSIC CAR";
    case MotionLabVehicleType::RaceCar:
        return "RACE CAR";
    case MotionLabVehicleType::Rocket:
        return "ROCKET";
    case MotionLabVehicleType::Helicopter:
        return "HELICOPTER";
    case MotionLabVehicleType::Hovercraft:
        return "HOVERCRAFT";
    case MotionLabVehicleType::TrackedTank:
        return "TRACKED TANK";
    }
    return "UNKNOWN";
}

} // namespace motion_lab
