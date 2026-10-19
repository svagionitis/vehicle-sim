#pragma once

#include <array>
#include <cstddef>
#include <vector>

#include "libs/VehicleSimCore/dynamics/IVehicleDynamics.h"
#include "libs/VehicleSimCore/math/Vec2.h"
#include "libs/VehicleSimCore/terrain/Heightfield1D.h"

namespace vehicle_sim::physics {
class RigidBody2D;
} // namespace vehicle_sim::physics

namespace vehicle_sim::terrain {
class DeformableTerrain;
} // namespace vehicle_sim::terrain

namespace vehicle_sim::dynamics {

/// @struct OffroadAxleConfig
/// @brief Configuration parameters for a heavy-duty 4WD truck suspension axle.
struct OffroadAxleConfig {
    double longitudinalOffset { 0.0 };     ///< Chassis X offset from CoM in meters.
    double restLength { 0.70 };            ///< Uncompressed suspension strut length in meters.
    double springStiffness { 65000.0 };    ///< Suspension spring constant k in N/m (firm offroad rate).
    double dampingCoeff { 7500.0 };        ///< Viscous damping coefficient c in N*s/m (well-damped).
    double bumpStopStiffness { 250000.0 }; ///< Non-linear elastomer bump stop stiffness in N/m^2.
    double bumpStopDistance { 0.25 };      ///< Strut deflection threshold activating bump stop.
    double tireRadius { 0.42 };            ///< Off-road knobby tire radius in meters.
    double tireWidth { 0.28 };             ///< Tire contact patch width in meters.
};

/// @struct OffroadTruckConfig
/// @brief Mechanical, suspension, and terramechanic parameters for the 4WD truck.
struct OffroadTruckConfig {
    double mass { 1450.0 };                ///< Chassis mass in kg.
    double momentOfInertia { 1200.0 };     ///< Chassis pitch rotational inertia in kg*m^2.
    double gravity { 9.81 };               ///< Downward acceleration in m/s^2.
    double airDensity { 1.225 };           ///< Atmospheric air density in kg/m^3.
    double dragCoefficient { 0.52 };       ///< Frontal aerodynamic drag coefficient Cd.
    double frontalArea { 2.1 };            ///< Projected frontal aerodynamic area in m^2.

    double maxDriveForce { 8500.0 };       ///< Total forward 4WD traction capacity in N.
    double maxBrakeForce { 11000.0 };      ///< Total 4WD service braking force capacity in N.
    double maxReverseForce { 4500.0 };     ///< Total reverse traction capacity in N.
    double jumpSpeed { 6.8 };              ///< Upward takeoff impulse speed in m/s.
    double jumpImpulse { 0.0 };            ///< Optional explicit vertical impulse in N*s (0 uses mass * jumpSpeed).
    double pitchStabGain { 1600.0 };       ///< Mid-air attitude stabilization torque gain.
    double antiRollTorqueGain { 7000.0 };  ///< Anti-roll pitch stiffness coupling axles.
    double groundFriction { 1.0 };         ///< Default ground surface friction coefficient.
    bool enableBekkerRutting { true };     ///< Enable Bekker terramechanics compaction drag.

    std::array<OffroadAxleConfig, 2> axles {
        OffroadAxleConfig{ -1.20, 0.70, 65000.0, 7500.0, 250000.0, 0.25, 0.42, 0.28 }, // Rear
        OffroadAxleConfig{  1.20, 0.70, 65000.0, 7500.0, 250000.0, 0.25, 0.42, 0.28 }  // Front
    };
};

/// @struct OffroadTruckInput
/// @brief Semantic driver control inputs for the 4WD desert truck.
struct OffroadTruckInput {
    double throttle { 0.0 };      ///< Normalized forward drive in [-1.0, 1.0] (positive = forward).
    double brake { 0.0 };         ///< Normalized brake in [0.0, 1.0].
    bool jump { false };          ///< Upward jump thruster command.
    bool firePrimary { false };   ///< Command to fire forward kinetic rocket.
    bool fireSecondary { false }; ///< Command to fire high-angle anti-air mortar.
};

/// @struct OffroadAxleState
/// @brief Diagnostic runtime suspension and terramechanic telemetry per axle.
struct OffroadAxleState {
    math::Vec2 worldPos { 0.0, 0.0 };      ///< Wheel hub center in world coordinates.
    math::Vec2 contactPoint { 0.0, 0.0 };  ///< Ground contact point on terrain.
    math::Vec2 surfaceNormal { 0.0, 1.0 }; ///< Ground unit normal at contact point.
    double compression { 0.0 };             ///< Strut displacement from rest in meters.
    double normalForce { 0.0 };             ///< Vertical ground reaction force in Newtons.
    double tractionForce { 0.0 };           ///< Tangential longitudinal force in Newtons.
    double soilSinkage { 0.0 };             ///< Bekker soil rut penetration depth in meters.
    double soilDragForce { 0.0 };           ///< Compaction resistance force opposing motion in N.
    bool isGrounded { false };              ///< Ground contact flag.
};

/// @class OffroadTruckDynamics
/// @brief 4WD desert racing truck dynamics coordinator.
/// @details Simulates independent long-travel spring-damper suspension with non-linear bump stops,
/// 50:50 4WD differential drive, Bekker terramechanic soil rut drag, and in-flight attitude stabilization.
class OffroadTruckDynamics final : public IVehicleDynamics {
public:
    static constexpr std::size_t kAxleCount { 2 };
    static constexpr std::size_t kRearAxle { 0 };
    static constexpr std::size_t kFrontAxle { 1 };

    /// @brief Default constructor using nominal 4WD desert truck parameters.
    OffroadTruckDynamics() noexcept;

    /// @brief Parameterized constructor using configuration settings.
    /// @param[in] config Vehicle mechanical and suspension configuration.
    explicit OffroadTruckDynamics(const OffroadTruckConfig& config) noexcept;

    /// @brief Destructor.
    ~OffroadTruckDynamics() noexcept override = default;

    /// @brief Advances 4WD truck dynamics simulation by timestep dt.
    /// @param[in,out] body Rigid body chassis receiving powertrain and suspension forces.
    /// @param[in] dt Timestep duration in seconds.
    void step(physics::RigidBody2D& body, double dt) noexcept override;

    /// @brief Resets transient state.
    void reset() noexcept override;

    /// @brief Checks if both axles are detached from the ground.
    /// @return True if truck is completely airborne.
    [[nodiscard]] bool isAirborne() const noexcept override;

    /// @brief Checks if at least one axle is in contact with the ground.
    /// @return True if vehicle is grounded.
    [[nodiscard]] bool isGrounded() const noexcept;

    /// @brief Sets active driver control inputs.
    /// @param[in] input Control input commands.
    void setInput(const OffroadTruckInput& input) noexcept;

    /// @brief Gets active driver control inputs.
    /// @return Constant reference to active OffroadTruckInput.
    [[nodiscard]] const OffroadTruckInput& getInput() const noexcept;

    /// @brief Sets terrain heightfield reference for elevation and slope queries.
    /// @param[in] terrain Pointer to continuous 1D heightfield.
    void setTerrain(const terrain::Heightfield1D* terrain) noexcept;

    /// @brief Gets terrain heightfield reference.
    /// @return Pointer to active continuous 1D heightfield.
    [[nodiscard]] const terrain::Heightfield1D* getTerrain() const noexcept;

    /// @brief Sets deformable terrain reference for Bekker rutting.
    /// @param[in] deformable Pointer to deformable terrain system.
    void setDeformable(terrain::DeformableTerrain* deformable) noexcept;

    /// @brief Gets deformable terrain reference.
    /// @return Pointer to active deformable terrain system.
    [[nodiscard]] terrain::DeformableTerrain* getDeformable() const noexcept;

    /// @brief Triggers an upward jump impulse.
    /// @param[in] speed Upward vertical speed in m/s (0 uses config default).
    void triggerJump(double speed = 0.0) noexcept;

    /// @brief Gets telemetry snapshot for an axle.
    /// @param[in] index Axle index (0 = Rear, 1 = Front).
    /// @return Reference to OffroadAxleState.
    [[nodiscard]] const OffroadAxleState& getAxleState(std::size_t index) const noexcept;

    /// @brief Gets vehicle configuration parameters.
    /// @return Constant reference to active OffroadTruckConfig.
    [[nodiscard]] const OffroadTruckConfig& getConfig() const noexcept;

    /// @brief Sets vehicle configuration parameters.
    /// @param[in] config Vehicle mechanical and suspension configuration.
    void setConfig(const OffroadTruckConfig& config) noexcept;

    /// @brief Gets current forward speed in m/s.
    /// @return Signed forward velocity along vehicle heading.
    [[nodiscard]] double getSpeed() const noexcept;

private:
    void resolveSuspension(physics::RigidBody2D& body, double dt) noexcept;
    void applyTraction(physics::RigidBody2D& body) noexcept;
    void applyAeroDrag(physics::RigidBody2D& body) noexcept;
    void applyAttitudeStabilizer(physics::RigidBody2D& body) noexcept;

    OffroadTruckConfig config_ {};
    OffroadTruckInput input_ {};
    const terrain::Heightfield1D* terrain_ { nullptr };
    terrain::DeformableTerrain* deformableTerrain_ { nullptr };
    std::array<OffroadAxleState, kAxleCount> axleStates_ {};

    double forwardSpeed_ { 0.0 };
    bool isAirborne_ { true };
    bool jumpTriggered_ { false };
    double jumpSpeedOverride_ { 0.0 };
    double jumpCooldown_ { 0.0 };
    bool jumpLatch_ { false };
};

} // namespace vehicle_sim::dynamics
