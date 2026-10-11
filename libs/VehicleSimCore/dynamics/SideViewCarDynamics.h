#pragma once

#include <array>
#include <cstddef>
#include <vector>

#include "libs/VehicleSimCore/dynamics/IVehicleDynamics.h"
#include "libs/VehicleSimCore/math/Vec2.h"

namespace vehicle_sim::physics {
class RigidBody2D;
} // namespace vehicle_sim::physics

namespace vehicle_sim::dynamics {

/// @enum FacingDirection
/// @brief Horizontal heading direction for side-view 2D platform driving.
enum class FacingDirection {
    Right, ///< Vehicle facing and driving towards positive X (+world X).
    Left ///< Vehicle facing and driving towards negative X (-world X).
};

/// @enum TurnaroundState
/// @brief Operational state of the 180-degree turnaround maneuver.
enum class TurnaroundState {
    Idle, ///< Normal driving, no turnaround in progress.
    Active ///< 180-degree turnaround spin actively executing.
};

/// @enum OilSlickState
/// @brief Surface traction and spinout state.
enum class OilSlickState {
    Normal, ///< Nominal surface traction available.
    Spinning ///< Loss-of-traction spinout active; inputs locked.
};

/// @struct SideViewPlatform
/// @brief Horizontal platform segment in world coordinates.
struct SideViewPlatform {
    double y { 0.0 }; ///< Surface elevation along Y axis in meters.
    double minX { -1e6 }; ///< Minimum horizontal boundary in meters.
    double maxX { 1e6 }; ///< Maximum horizontal boundary in meters.
    double friction { 1.0 }; ///< Surface friction coefficient multiplier.
    bool oneWay { true }; ///< True if platform allows upward pass-through.
};

/// @struct SideViewCarInput
/// @brief Semantic driver control inputs for the side-view platform coordinator.
struct SideViewCarInput {
    double throttle { 0.0 }; ///< Drive throttle in [-1.0, 1.0] (positive = facing forward).
    double brake { 0.0 }; ///< Service brake input in [0.0, 1.0].
    bool jump { false }; ///< Jump impulse trigger command.
    bool turnaround { false }; ///< 180-degree turnaround trigger command.
};

/// @struct SideViewCarConfig
/// @brief Mechanical, suspension, and stunt parameters for side-view dynamics.
struct SideViewCarConfig {
    double wheelbase { 2.2 }; ///< Distance between front and rear contact points in meters.
    double wheelRadius { 0.35 }; ///< Wheel radius defining contact elevation in meters.
    double mass { 1000.0 }; ///< Vehicle mass in kilograms.
    double gravity { 9.81 }; ///< Downward gravitational acceleration in m/s^2.

    double maxDriveForce { 6000.0 }; ///< Forward traction force capacity in Newtons.
    double maxBrakeForce { 9000.0 }; ///< Service braking deceleration force in Newtons.
    double maxReverseForce { 3500.0 }; ///< Reverse traction force capacity in Newtons.

    double linearDrag { 0.30 }; ///< Aerodynamic drag coefficient in kg/m.
    double rollingResistance { 20.0 }; ///< Rolling resistance damping in N/(m/s).

    double jumpSpeed { 7.5 }; ///< Upward vertical takeoff velocity in m/s.
    double jumpImpulse { 0.0 }; ///< Optional explicit vertical impulse in N*s (0 uses mass * jumpSpeed).

    double turnaroundTorque { 25000.0 }; ///< Rapid angular torque applied during 180-deg flip in N*m.
    double turnaroundDuration { 0.20 }; ///< Duration of 180-deg turnaround spin in seconds (<=0 for instant).

    double oilSlickDuration { 1.5 }; ///< Loss-of-traction spinout duration in seconds.
    double oilSlickFriction { 0.05 }; ///< Residual friction multiplier during spinout.
    double oilSlickTorque { 4000.0 }; ///< Spinout perturbation torque in N*m.

    double groundRestitution { 0.05 }; ///< Platform landing normal restitution.
    double platformFriction { 0.85 }; ///< Platform Coulomb contact friction.
};

/// @struct SideViewWheelTelemetry
/// @brief Diagnostic telemetry snapshot for an individual side-view wheel.
struct SideViewWheelTelemetry {
    math::Vec2 localOffset { 0.0, 0.0 }; ///< Local chassis offset relative to center of mass.
    math::Vec2 worldPosition { 0.0, 0.0 }; ///< Contact position in world coordinates.
    math::Vec2 velocity { 0.0, 0.0 }; ///< Velocity at wheel contact point in m/s.
    double normalForce { 0.0 }; ///< Normal ground reaction force in Newtons.
    double tractionForce { 0.0 }; ///< Longitudinal driving or braking force in Newtons.
    bool isGrounded { false }; ///< True if wheel is currently in contact with a platform.
};

/// @class SideViewCarDynamics
/// @brief Side-view 2D platform vehicle dynamics coordinator (City Connection style).
/// @details Simulates horizontal platform driving, vertical gravity integration, platform
/// contact normal impulses, jump takeoff impulses, instant or timed 180-degree turnaround
/// maneuvers, and oil slick loss-of-traction spinouts.
///
/// ASCII Diagram:
/// @code
///          +-----------------------+
///          |   Chassis (RigidBody) |
///          +-----------+-----------+
///             O                 O
///        Rear Wheel        Front Wheel
///     ====================================  Platform (Y = Y_plat)
/// @endcode
///
/// Mermaid Diagram:
/// @code
/// @verbatim
/// graph TD
///     Grounded[Grounded on Platform] -->|triggerJump| Airborne[Airborne Flight]
///     Airborne -->|Normal Contact Impulse| Grounded
///     Grounded -->|triggerOilSlick| Spinout[Oil Slick Spinout]
///     Spinout -->|Timer Expired| Grounded
///     Grounded -->|triggerTurnaround| Turnaround[180 Turnaround Spin]
///     Turnaround -->|Turnaround Done| Grounded
/// @endverbatim
/// @endcode
class SideViewCarDynamics final : public IVehicleDynamics {
public:
    static constexpr std::size_t kWheelCount { 2 };
    static constexpr std::size_t kRearWheel { 0 };
    static constexpr std::size_t kFrontWheel { 1 };

    /// @brief Default constructor with baseline platformer car configuration.
    SideViewCarDynamics() noexcept;

    /// @brief Parameterized constructor using configuration settings.
    /// @param[in] config Vehicle configuration parameters.
    explicit SideViewCarDynamics(const SideViewCarConfig& config) noexcept;

    /// @brief Destructor.
    ~SideViewCarDynamics() noexcept override = default;

    /// @brief Advances side-view platform car dynamics simulation by timestep dt.
    /// @details Evaluates platform contacts, normal impulses, driving forces, aerodynamics,
    /// jump impulses, turnaround rotations, and oil slick spinouts.
    /// @param[in,out] body Target rigid body to receive powertrain, contact, and aero forces.
    /// @param[in] dt Timestep duration in seconds.
    void step(physics::RigidBody2D& body, double dt) noexcept override;

    /// @brief Resets transient states.
    /// @details Resets facing direction to Right, clears airborne/spinout/turnaround timers,
    /// and resets wheel telemetry without modifying rigid body position or momentum.
    void reset() noexcept override;

    /// @brief Queries whether vehicle is currently airborne.
    /// @return True if vehicle is detached from all platform surfaces.
    [[nodiscard]] bool isAirborne() const noexcept override;

    /// @brief Queries whether vehicle is grounded on a platform.
    /// @return True if at least one wheel is resting on a platform.
    [[nodiscard]] bool isGrounded() const noexcept;

    /// @brief Queries whether vehicle is in an oil slick spinout.
    /// @return True if oil slick spinout is actively spinning.
    [[nodiscard]] bool isSpinningOut() const noexcept;

    /// @brief Queries whether a 180-degree turnaround maneuver is in progress.
    /// @return True if vehicle is actively rotating during a turnaround.
    [[nodiscard]] bool isTurningAround() const noexcept;

    /// @brief Gets active facing direction.
    /// @return Current FacingDirection (Right or Left).
    [[nodiscard]] FacingDirection getFacingDirection() const noexcept;

    /// @brief Explicitly sets horizontal facing direction.
    /// @param[in] direction Desired facing direction.
    void setFacingDirection(FacingDirection direction) noexcept;

    /// @brief Gets active turnaround maneuver state.
    /// @return TurnaroundState enum value.
    [[nodiscard]] TurnaroundState getTurnaroundState() const noexcept;

    /// @brief Gets active oil slick traction state.
    /// @return OilSlickState enum value.
    [[nodiscard]] OilSlickState getOilSlickState() const noexcept;

    /// @brief Gets remaining duration of active oil slick spinout.
    /// @return Remaining time in seconds.
    [[nodiscard]] double getOilSlickTimer() const noexcept;

    /// @brief Gets remaining duration of active 180-degree turnaround.
    /// @return Remaining time in seconds.
    [[nodiscard]] double getTurnaroundTimer() const noexcept;

    /// @brief Gets current forward speed along facing axis in m/s.
    /// @return Forward speed scalar in m/s.
    [[nodiscard]] double getSpeed() const noexcept;

    /// @brief Sets semantic driver control inputs.
    /// @param[in] input Control input commands.
    void setInput(const SideViewCarInput& input) noexcept;

    /// @brief Gets active driver control inputs.
    /// @return Active SideViewCarInput reference.
    [[nodiscard]] const SideViewCarInput& getInput() const noexcept;

    /// @brief Gets vehicle configuration.
    /// @return Active SideViewCarConfig reference.
    [[nodiscard]] const SideViewCarConfig& getConfig() const noexcept;

    /// @brief Sets vehicle configuration and updates wheel geometry.
    /// @param[in] config New configuration parameters.
    void setConfig(const SideViewCarConfig& config) noexcept;

    /// @brief Triggers an upward jump takeoff impulse.
    /// @param[in] speed Upward vertical takeoff speed in m/s (<= 0 uses config jumpSpeed).
    void triggerJump(double speed = 0.0) noexcept;

    /// @brief Initiates a 180-degree turnaround maneuver.
    /// @param[in] instant True to flip heading instantaneously, false to use configured duration.
    void triggerTurnaround(bool instant = false) noexcept;

    /// @brief Triggers an oil slick loss-of-traction spinout.
    /// @param[in] duration Spinout duration in seconds (<= 0 uses config oilSlickDuration).
    void triggerOilSlick(double duration = 0.0) noexcept;

    /// @brief Registers a platform segment for contact evaluation.
    /// @param[in] platform Platform specification.
    void addPlatform(const SideViewPlatform& platform);

    /// @brief Helper to add a horizontal platform segment.
    /// @param[in] y Surface elevation in meters.
    /// @param[in] minX Minimum horizontal coordinate in meters.
    /// @param[in] maxX Maximum horizontal coordinate in meters.
    /// @param[in] friction Surface friction multiplier.
    /// @param[in] oneWay True if platform allows upward pass-through.
    void addPlatform(double y, double minX, double maxX, double friction = 1.0, bool oneWay = true);

    /// @brief Removes all registered platforms.
    void clearPlatforms() noexcept;

    /// @brief Gets count of registered platforms.
    /// @return Number of registered platform segments.
    [[nodiscard]] std::size_t getPlatformCount() const noexcept;

    /// @brief Gets registered platform segments.
    /// @return Constant reference to vector of platforms.
    [[nodiscard]] const std::vector<SideViewPlatform>& getPlatforms() const noexcept;

    /// @brief Gets diagnostic telemetry for a specific wheel.
    /// @param[in] index Wheel index in [0, 1].
    /// @return SideViewWheelTelemetry reference.
    [[nodiscard]] const SideViewWheelTelemetry& getWheelTelemetry(std::size_t index) const noexcept;

    /// @brief Sets global terrain friction modifier.
    /// @param[in] gripMultiplier Ground friction scale factor.
    void setSurfaceModifier(double gripMultiplier) noexcept;

    /// @brief Gets global terrain friction modifier.
    /// @return Ground friction scale factor.
    [[nodiscard]] double getSurfaceModifier() const noexcept;

private:
    void updateWheelLayout() noexcept;
    void resolvePlatformContact(physics::RigidBody2D& body, double dt) noexcept;
    void updateTurnaround(physics::RigidBody2D& body, double dt) noexcept;
    void updateOilSlick(physics::RigidBody2D& body, double dt) noexcept;
    void applyDriveForces(physics::RigidBody2D& body) noexcept;

    SideViewCarConfig mConfig { };
    SideViewCarInput mInput { };

    std::vector<SideViewPlatform> mPlatforms { };
    std::array<SideViewWheelTelemetry, kWheelCount> mWheels { };

    FacingDirection mFacingDirection { FacingDirection::Right };
    TurnaroundState mTurnaroundState { TurnaroundState::Idle };
    OilSlickState mOilSlickState { OilSlickState::Normal };

    double mGlobalGripModifier { 1.0 };
    double mTurnaroundTimer { 0.0 };
    double mOilSlickTimer { 0.0 };
    double mSpeed { 0.0 };

    bool mJumpTriggered { false };
    double mJumpSpeedOverride { 0.0 };
    bool mTurnaroundTriggered { false };
    bool mInstantTurnaround { false };
};

} // namespace vehicle_sim::dynamics
