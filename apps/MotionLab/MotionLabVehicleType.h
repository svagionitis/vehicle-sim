#pragma once

#include <string_view>

namespace motion_lab {

/// @enum MotionLabVehicleType
/// @brief Available vehicle simulation models in Motion Lab.
enum class MotionLabVehicleType {
    ClassicCar,      ///< F1: Pat Kerr 1996 classic car with velocity-decomposed tire slip.
    RaceCar,         ///< F2: High-performance motorsport race car with downforce and DRS.
    Rocket,          ///< F3: 2D space rocket with gimbaled vector thrust and RCS attitude control.
    Helicopter,      ///< F4: VTOL rotorcraft with collective pitch and ground effect.
    Hovercraft,      ///< F5: Air-cushion vehicle with vectored thrust and low surface friction.
    TrackedTank,     ///< F6: Dual-track armored tank with skid-steer and cannon recoil.
    AntiGravity,     ///< F7: Futuristic maglev racer with repulsor hover and airbrakes.
    DriftCar,        ///< F8: High-steering-lock drift vehicle with thermal tire degradation.
    MonsterTruck,    ///< F9: Semi-realistic 4-wheel drive truck with engine torque curves.
    SemiTruck,       ///< F10: Articulated semi-truck with fifth-wheel hitch and trailer.
    Forklift,        ///< F11: Rear-wheel steering industrial forklift with elevating mast.
    QuadcopterDrone  ///< F12: 4-rotor aerial drone with closed-loop PID attitude control.
};

/// @brief Gets human-readable uppercase name for a vehicle type.
/// @param[in] type Target vehicle model enum.
/// @return Display string view.
[[nodiscard]] std::string_view getVehicleTypeName(MotionLabVehicleType type) noexcept;

} // namespace motion_lab
