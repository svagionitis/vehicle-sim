#pragma once

#include <string_view>

namespace motion_lab {

/// @enum MotionLabVehicleType
/// @brief Available vehicle simulation models in Motion Lab.
enum class MotionLabVehicleType {
    ClassicCar,   ///< Pat Kerr 1996 classic car with velocity-decomposed tire slip.
    RaceCar,      ///< High-performance motorsport race car with downforce and DRS.
    Rocket,       ///< 2D space rocket with gimbaled vector thrust and RCS attitude control.
    Helicopter,   ///< VTOL rotorcraft with collective pitch and ground effect.
    Hovercraft,   ///< Air-cushion vehicle with vectored thrust and low surface friction.
    TrackedTank   ///< Dual-track armored tank with skid-steer and cannon recoil.
};

/// @brief Gets human-readable uppercase name for a vehicle type.
/// @param[in] type Target vehicle model enum.
/// @return Display string view.
[[nodiscard]] std::string_view getVehicleTypeName(MotionLabVehicleType type) noexcept;

} // namespace motion_lab
