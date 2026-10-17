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
    case MotionLabVehicleType::AntiGravity:
        return "ANTI-GRAVITY";
    case MotionLabVehicleType::DriftCar:
        return "DRIFT CAR";
    case MotionLabVehicleType::MonsterTruck:
        return "MONSTER TRUCK";
    case MotionLabVehicleType::SemiTruck:
        return "SEMI-TRUCK";
    case MotionLabVehicleType::Forklift:
        return "FORKLIFT";
    case MotionLabVehicleType::QuadcopterDrone:
        return "DRONE (UAV)";
    }
    return "UNKNOWN";
}

} // namespace motion_lab
