#pragma once

#include <array>
#include <cstddef>

#include "libs/VehicleSimCore/dynamics/IVehicleDynamics.h"
#include "libs/VehicleSimCore/math/Vec2.h"

namespace vehicle_sim::physics {
class RigidBody2D;
} // namespace vehicle_sim::physics

namespace vehicle_sim::terrain {
class Heightfield1D;
} // namespace vehicle_sim::terrain

namespace vehicle_sim::dynamics {

/// @struct EarthRoverWheelConfig
/// @brief Geometric and mechanical parameters for an individual rover suspension axle.
struct EarthRoverWheelConfig {
    double longitudinalOffset { 0.0 }; ///< Chassis X offset from CoM in meters.
    double restLength { 0.60 }; ///< Uncompressed suspension strut length in meters.
    double springStiffness { 24000.0 }; ///< Suspension spring constant k in N/m.
    double dampingCoeff { 1800.0 }; ///< Viscous damping coefficient c in N*s/m.
    double wheelRadius { 0.35 }; ///< Wheel radius in meters.
};

/// @struct EarthRoverConfig
/// @brief Full mechanical, aerodynamic, and suspension configuration for the Earth rover.
struct EarthRoverConfig {
    double mass { 850.0 }; ///< Chassis mass in kg.
    double gravity { 9.81 }; ///< Downward acceleration in m/s^2.
    double airDensity { 1.225 }; ///< Atmospheric air density in kg/m^3.
    double dragCoefficient { 0.42 }; ///< Frontal aerodynamic drag coefficient Cd.
    double frontalArea { 1.6 }; ///< Projected frontal aerodynamic area in m^2.

    double maxDriveForce { 5500.0 }; ///< Forward longitudinal traction capacity in N.
    double maxBrakeForce { 8000.0 }; ///< Longitudinal braking deceleration force in N.
    double maxReverseForce { 2500.0 }; ///< Reverse longitudinal traction capacity in N.
    double jumpSpeed { 8.5 }; ///< Upward takeoff impulse speed in m/s.
    double jumpImpulse { 0.0 }; ///< Optional explicit impulse in N*s (0 uses mass * jumpSpeed).
    double pitchStabGain { 3500.0 }; ///< Mid-air attitude stabilization torque gain.
    double pitchDampGain { 2500.0 }; ///< Mid-air angular velocity damping gain.

    double groundFriction { 0.90 }; ///< Baseline Coulomb tire-ground friction coefficient.

    std::array<EarthRoverWheelConfig, 3> wheels {
        EarthRoverWheelConfig { -1.10, 0.60, 24000.0, 1800.0, 0.35 }, // Rear
        EarthRoverWheelConfig { 0.00, 0.60, 22000.0, 1600.0, 0.35 }, // Middle
        EarthRoverWheelConfig { 1.10, 0.60, 24000.0, 1800.0, 0.35 } // Front
    };
};

/// @struct EarthRoverInput
/// @brief Semantic driver control inputs for the Earth rover.
struct EarthRoverInput {
    double throttle { 0.0 }; ///< Drive throttle in [-1.0, 1.0] (positive = forward).
    double brake { 0.0 }; ///< Service brake input in [0.0, 1.0].
    bool jump { false }; ///< Jump impulse trigger command.
};

/// @struct EarthRoverWheelState
/// @brief Runtime suspension and contact telemetry for an axle.
struct EarthRoverWheelState {
    math::Vec2 worldPos { 0.0, 0.0 }; ///< Wheel contact center in world space.
    math::Vec2 contactPoint { 0.0, 0.0 }; ///< Surface contact point on terrain in world space.
    math::Vec2 surfaceNormal { 0.0, 1.0 }; ///< Upward terrain normal at contact point.
    double compression { 0.0 }; ///< Strut displacement from rest in meters.
    double normalForce { 0.0 }; ///< Vertical ground reaction force in Newtons.
    double tractionForce { 0.0 }; ///< Tangential longitudinal force in Newtons.
    bool isGrounded { false }; ///< Ground contact engagement flag.
};

/// @class EarthRoverDynamics
/// @brief 3-axle articulated suspension rover dynamics coordinator (Moon Patrol Earth edition).
/// @details Simulates independent multi-wheel spring-damper suspension over continuous
/// 1D heightfields, jump takeoff impulses, Earth atmospheric drag, and mid-air attitude damping.
///
/// ```
///  ASCII Diagram: 3-Axle Rover Suspension
///
///                 Chassis Center of Mass (CoM)
///                             |
///                             v
///            +---------------------------------+
///            |      Earth Patrol Chassis       |
///            +----+-------------+-------------+
///                 |             |             |
///            k,c  |        k,c  |        k,c  |  (Spring-Damper Struts)  |
///                / \           / \           / \                        |
///               |   |         |   |         |   |                       |
///                \ /           \ /           \ /                        |
///                 |             |             |                         |
///              (Rear)       (Middle)       (Front)
///              [ W0 ]        [ W1 ]        [ W2 ]
///          ======/=============\============\=======  Terrain Surface h(x)
/// ```
///
/// ```mermaid
/// graph TD
///     Grounded[Grounded on Terrain] -->|triggerJump| Airborne[Airborne Leap]
///     Airborne -->|Earth Gravity 9.81 m/s^2| Descent[Parabolic Descent]
///     Descent -->|Wheel Ground Contact| Grounded
///     Airborne -->|Attitude Stabilization| Upright[Pitch Leveling]
/// ```
class EarthRoverDynamics final : public IVehicleDynamics {
public:
    static constexpr std::size_t kWheelCount { 3 };
    static constexpr std::size_t kRearWheel { 0 };
    static constexpr std::size_t kMiddleWheel { 1 };
    static constexpr std::size_t kFrontWheel { 2 };

    /// @brief Default constructor using nominal Earth rover parameters.
    EarthRoverDynamics() noexcept;

    /// @brief Parameterized constructor using configuration settings.
    /// @param[in] config Vehicle mechanical and suspension configuration.
    explicit EarthRoverDynamics(const EarthRoverConfig& config) noexcept;

    /// @brief Destructor.
    ~EarthRoverDynamics() noexcept override = default;

    /// @brief Advances rover dynamics simulation by timestep dt.
    /// @param[in,out] body Rigid body chassis receiving powertrain and suspension forces.
    /// @param[in] dt Timestep duration in seconds.
    void step(physics::RigidBody2D& body, double dt) noexcept override;

    /// @brief Resets transient state.
    void reset() noexcept override;

    /// @brief Checks if all wheels are detached from the ground.
    /// @return True if vehicle is completely airborne.
    [[nodiscard]] bool isAirborne() const noexcept override;

    /// @brief Checks if at least one wheel is in contact with the ground.
    /// @return True if grounded on terrain.
    [[nodiscard]] bool isGrounded() const noexcept;

    /// @brief Sets active driver input commands.
    /// @param[in] input Control input commands.
    void setInput(const EarthRoverInput& input) noexcept;

    /// @brief Gets active driver input commands.
    /// @return Constant reference to active EarthRoverInput.
    [[nodiscard]] const EarthRoverInput& getInput() const noexcept;

    /// @brief Sets terrain heightfield reference for ground contact evaluation.
    /// @param[in] terrain Pointer to continuous 1D heightfield (or nullptr).
    void setTerrain(const terrain::Heightfield1D* terrain) noexcept;

    /// @brief Gets registered terrain heightfield reference.
    /// @return Pointer to registered Heightfield1D or nullptr.
    [[nodiscard]] const terrain::Heightfield1D* getTerrain() const noexcept;

    /// @brief Triggers an upward jump impulse.
    /// @param[in] speed Upward vertical speed in m/s (0.0 uses config default).
    void triggerJump(double speed = 0.0) noexcept;

    /// @brief Gets wheel telemetry snapshot.
    /// @param[in] index Axle index in [0, 2].
    /// @return Reference to EarthRoverWheelState.
    [[nodiscard]] const EarthRoverWheelState& getWheelState(std::size_t index) const noexcept;

    /// @brief Gets vehicle configuration parameters.
    /// @return Constant reference to active EarthRoverConfig.
    [[nodiscard]] const EarthRoverConfig& getConfig() const noexcept;

    /// @brief Sets vehicle configuration parameters.
    /// @param[in] config New configuration parameters.
    void setConfig(const EarthRoverConfig& config) noexcept;

    /// @brief Gets forward speed along chassis longitudinal axis in m/s.
    /// @return Forward speed scalar in m/s.
    [[nodiscard]] double getSpeed() const noexcept;

private:
    /// @brief Resolves suspension ground contacts and computes normal reactions.
    /// @param[in,out] body Target chassis rigid body.
    /// @param[in] dt Timestep duration in seconds.
    void resolveSuspension(physics::RigidBody2D& body, double dt) noexcept;

    /// @brief Applies longitudinal traction and braking forces along wheel contacts.
    /// @param[in,out] body Target chassis rigid body.
    void applyTraction(physics::RigidBody2D& body) noexcept;

    /// @brief Applies quadratic aerodynamic drag forces opposing velocity.
    /// @param[in,out] body Target chassis rigid body.
    void applyAeroDrag(physics::RigidBody2D& body) noexcept;

    /// @brief Applies mid-air attitude leveling and angular rate damping.
    /// @param[in,out] body Target chassis rigid body.
    void applyAttitudeStabilizer(physics::RigidBody2D& body) noexcept;

    EarthRoverConfig config_ {};
    EarthRoverInput input_ {};
    const terrain::Heightfield1D* terrain_ { nullptr };
    std::array<EarthRoverWheelState, kWheelCount> wheelStates_ {};

    double forwardSpeed_ { 0.0 };
    bool isAirborne_ { true };
    bool jumpTriggered_ { false };
    double jumpSpeedOverride_ { 0.0 };
};

} // namespace vehicle_sim::dynamics
