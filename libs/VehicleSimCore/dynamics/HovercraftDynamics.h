#pragma once

#include <cmath>

#include "libs/VehicleSimCore/dynamics/IVehicleDynamics.h"
#include "libs/VehicleSimCore/math/Vec2.h"

namespace vehicle_sim::physics {
class RigidBody2D;
} // namespace vehicle_sim::physics

namespace vehicle_sim::dynamics {

/// @struct HovercraftConfig
/// @brief Configuration parameters for 2D air-cushion vehicle (hovercraft) dynamics.
/// @details Defines lift-fan cushion compliance, vectored thrust propulsion, rudder deflection,
/// amphibious surface friction, low-friction drifting, and aerodynamic drag.
struct HovercraftConfig {
    double mass { 1200.0 }; ///< Hull and payload structural mass in kg.
    double length { 4.0 }; ///< Hull total length in meters.
    double width { 2.2 }; ///< Hull beam width in meters.
    double hoverHeight { 0.35 }; ///< Nominal equilibrium hover cushion clearance in meters.
    double cushionStiffness { 35000.0 }; ///< Air cushion vertical suspension spring rate in N/m.
    double cushionDamping { 4500.0 }; ///< Air cushion vertical damping coefficient in N*s/m.

    double maxThrust { 6000.0 }; ///< Peak rear propulsion thruster force in Newtons.
    double reverseThrustRatio { 0.45 }; ///< Reverse thruster force multiplier in [0.0, 1.0].
    double maxRudderAngle { 0.523599 }; ///< Maximum rudder deflection angle in radians (~30 deg).
    double rudderSlewRate { 3.0 }; ///< Rudder steering slew rate in rad/s.
    math::Vec2 thrusterOffset { -1.8, 0.0 }; ///< Rear fan nozzle mount position in local chassis coordinates.

    double skirtFriction { 0.75 }; ///< High contact friction coefficient when skirt drags on ground.
    double driftFriction { 0.04 }; ///< Low sliding friction coefficient while hovering over terrain.
    double lateralGrip { 400.0 }; ///< Minor lateral aerodynamic/skirt resistance in N*s/m.
    double airDrag { 12.0 }; ///< Quadratic air resistance coefficient in N/(m/s)^2.
    double angularDamping { 650.0 }; ///< Aerodynamic yaw rotation damping in N*m*s.
    math::Vec2 gravity { 0.0, -9.80665 }; ///< Ambient gravity vector in m/s^2.
    double surfaceFriction { 1.0 }; ///< Ambient terrain friction modifier (e.g., ice vs tarmac).
};

/// @struct HovercraftInput
/// @brief Pilot control commands for air-cushion vehicle dynamics.
struct HovercraftInput {
    double throttle { 0.0 }; ///< Commanded propulsion fan throttle in [-1.0, 1.0].
    double steering { 0.0 }; ///< Commanded rudder steering deflection in [-1.0, 1.0].
    bool liftFanActive { true }; ///< Lift fan power flag maintaining cushion air pressure.
    double groundDistance { 0.35 }; ///< Terrain clearance sensor measurement in meters.
};

/// @struct HovercraftTelemetry
/// @brief Diagnostic snapshot of hovercraft cushion and propulsion dynamics states.
struct HovercraftTelemetry {
    double forwardSpeed { 0.0 }; ///< Longitudinal velocity along hull heading in m/s.
    double driftSpeed { 0.0 }; ///< Lateral drift velocity across hull beam in m/s.
    double cushionForce { 0.0 }; ///< Current vertical lift cushion force in Newtons.
    double thrustForce { 0.0 }; ///< Current longitudinal thrust force output in Newtons.
    double rudderAngle { 0.0 }; ///< Current active rudder steering angle in radians.
    bool isHovering { true }; ///< True if lift cushion is actively supporting chassis.
    bool isSkirtScraping { false }; ///< True if bottom skirt is dragging against ground terrain.
    double driftAngle { 0.0 }; ///< Slip angle between hull heading and flight velocity in radians.
    double altitude { 0.35 }; ///< Current ground clearance altitude in meters.
};

/// @class HovercraftDynamics
/// @brief 2D air-cushion vehicle (hovercraft) dynamics coordinator.
/// @details Models lift-fan ground cushion vertical equilibrium, vectored thrust steering,
/// ultra-low surface friction drifting, skirt drag transitions, and amphibious terrain scaling.
class HovercraftDynamics final : public IVehicleDynamics {
public:
    /// @brief Default constructor using baseline hovercraft configuration.
    HovercraftDynamics() noexcept;

    /// @brief Parameterized constructor with custom hovercraft configuration.
    /// @param[in] config Hovercraft configuration parameters.
    explicit HovercraftDynamics(const HovercraftConfig& config) noexcept;

    /// @brief Advances hovercraft simulation step by timestep dt.
    /// @details Computes air cushion lift, vectors rear propulsion thrust with rudder slew,
    /// applies low-friction drift forces, and updates target rigid body.
    /// @param[in,out] body Target rigid body receiving forces and moments.
    /// @param[in] dt Timestep duration in seconds.
    void step(physics::RigidBody2D& body, double dt) noexcept override;

    /// @brief Resets transient control inputs and rudder deflection.
    void reset() noexcept override;

    /// @brief Queries whether the hovercraft is airborne above its cushion.
    /// @return True if altitude significantly exceeds nominal hover height.
    [[nodiscard]] bool isAirborne() const noexcept override;

    /// @brief Gets active hovercraft configuration.
    /// @return Const reference to active HovercraftConfig.
    [[nodiscard]] const HovercraftConfig& getConfig() const noexcept;

    /// @brief Updates active hovercraft configuration.
    /// @param[in] config New configuration parameters.
    void setConfig(const HovercraftConfig& config) noexcept;

    /// @brief Gets current pilot control inputs.
    /// @return Const reference to active HovercraftInput.
    [[nodiscard]] const HovercraftInput& getInput() const noexcept;

    /// @brief Sets pilot control inputs.
    /// @param[in] input New pilot control commands.
    void setInput(const HovercraftInput& input) noexcept;

    /// @brief Sets terrain surface traction multiplier.
    /// @param[in] friction Surface friction multiplier.
    void setSurfaceFriction(double friction) noexcept;

    /// @brief Gets diagnostic telemetry snapshot.
    /// @return Const reference to HovercraftTelemetry.
    [[nodiscard]] const HovercraftTelemetry& getTelemetry() const noexcept;

    /// @brief Queries whether the vehicle is actively hovering on cushion.
    /// @return True if lift fan is active and ground clearance is positive.
    [[nodiscard]] bool isHovering() const noexcept;

    /// @brief Gets longitudinal forward velocity.
    /// @return Forward speed in m/s.
    [[nodiscard]] double getSpeed() const noexcept;

    /// @brief Gets lateral sideways drift velocity.
    /// @return Drift velocity in m/s.
    [[nodiscard]] double getDriftSpeed() const noexcept;

    /// @brief Gets active rudder deflection angle.
    /// @return Rudder angle in radians.
    [[nodiscard]] double getRudderAngle() const noexcept;

private:
    HovercraftConfig mConfig { };
    HovercraftInput mInput { };
    HovercraftTelemetry mTelemetry { };
    double mRudderAngle { 0.0 };
};

} // namespace vehicle_sim::dynamics
