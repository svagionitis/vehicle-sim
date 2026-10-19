#pragma once

#include <array>
#include <cmath>
#include <cstdint>

#include "libs/VehicleSimCore/dynamics/IVehicleDynamics.h"
#include "libs/VehicleSimCore/dynamics/PacejkaTireModel.h"
#include "libs/VehicleSimCore/math/Vec2.h"

namespace vehicle_sim::physics {
class RigidBody2D;
} // namespace vehicle_sim::physics

namespace vehicle_sim::dynamics {

/// @enum QuadDiffMode
/// @brief Rear axle differential operational mode for QuadBikeDynamics.
enum class QuadDiffMode : std::uint8_t {
    LockedSpool = 0, ///< Solid spool drive axle forcing equal wheel speeds (tire scrub in corners).
    OpenDifferential = 1 ///< Open differential dividing drive torque 50/50 with independent wheel speeds.
};

/// @struct QuadBikeConfig
/// @brief Physical, balloon tire, suspension, and rider parameters for QuadBikeDynamics.
struct QuadBikeConfig {
    double chassisMass { 210.0 }; ///< Dry mass of quad bike chassis, engine, and frame in kg.
    double riderMass { 78.0 }; ///< Active rider body mass in kg.
    double yawInertia { 95.0 }; ///< Yaw moment of inertia in kg*m^2.
    double rollInertia { 38.0 }; ///< Roll moment of inertia in kg*m^2.

    double wheelbase { 1.28 }; ///< Longitudinal wheelbase L_wb in meters.
    double distFrontAxle { 0.68 }; ///< Distance from static COG to front axle in meters.
    double trackWidthFront { 1.18 }; ///< Front track width in meters.
    double trackWidthRear { 1.12 }; ///< Rear track width in meters.
    double cogHeightStatic { 0.52 }; ///< Static composite Center of Gravity height in meters.

    // Compliant Balloon Tire Parameters
    double tireRadiusFree { 0.32 }; ///< Unloaded balloon tire outer radius in meters (25-inch tire).
    double tireCarcassStiffness { 28000.0 }; ///< Radial carcass spring rate k_balloon in N/m.
    double tireCarcassDamping { 1200.0 }; ///< Radial carcass viscous damping c_balloon in N*s/m.
    double nominalPatchArea { 0.045 }; ///< Nominal ground contact patch area A_0 in m^2.
    double footprintExpansionGain { 1.85 }; ///< Footprint area expansion multiplier eta_balloon.
    double softSoilGripGain { 0.35 }; ///< Soft soil flotation friction boost multiplier zeta_balloon.

    // Suspension & Rear Solid Axle
    double frontSuspensionStiffness { 14000.0 }; ///< Front double-wishbone spring rate in N/m.
    double frontSuspensionDamping { 1500.0 }; ///< Front shock absorber damping in N*s/m.
    double rearSuspensionStiffness { 22000.0 }; ///< Rear swingarm spring rate in N/m.
    double rearSuspensionDamping { 2100.0 }; ///< Rear mono-shock damping in N*s/m.
    QuadDiffMode rearDiffMode { QuadDiffMode::LockedSpool }; ///< Active rear axle differential mode.

    // Steering & Powertrain
    double maxSteerAngle { 0.56 }; ///< Maximum front spindle steer angle in radians (~32 degrees).
    double maxDriveForce { 4800.0 }; ///< Peak 4x4 / rear-drive tractive force in Newtons.
    double maxFrontBrakeForce { 3200.0 }; ///< Front hydraulic disc brake force capacity in Newtons.
    double maxRearBrakeForce { 3800.0 }; ///< Rear solid axle hydraulic disc brake capacity in Newtons.

    // Active Rider Torso Shift Capacity
    double riderMaxShiftX { 0.30 }; ///< Maximum longitudinal rider torso displacement in meters (+/-).
    double riderMaxShiftY { 0.35 }; ///< Maximum lateral rider torso displacement in meters (+/-).

    PacejkaConfig frontTirePacejka {}; ///< Front balloon tire Magic Formula parameters.
    PacejkaConfig rearTirePacejka {}; ///< Rear balloon tire Magic Formula parameters.
};

/// @struct QuadBikeInput
/// @brief Rider control commands for all-terrain quad bike dynamics.
struct QuadBikeInput {
    double throttle { 0.0 }; ///< Engine drive throttle in range [0.0, 1.0].
    double frontBrake { 0.0 }; ///< Front handlebar brake lever in range [0.0, 1.0].
    double rearBrake { 0.0 }; ///< Rear foot brake pedal in range [0.0, 1.0].
    double steer { 0.0 }; ///< Handlebar steer command in range [-1.0, 1.0] (+ right, - left).
    double riderLeanPitch { 0.0 }; ///< Rider longitudinal torso shift in [-1.0, 1.0] (+ forward, - back).
    double riderLeanRoll { 0.0 }; ///< Rider lateral torso shift in [-1.0, 1.0] (+ right, - left).
};

/// @struct QuadWheelState
/// @brief Contact, deflection, and force telemetry for a single quad bike wheel.
struct QuadWheelState {
    double normalLoad { 0.0 }; ///< Dynamic vertical normal load in Newtons.
    double carcassDeflection { 0.0 }; ///< Radial carcass deflection Delta z in meters.
    double contactArea { 0.0 }; ///< Effective contact patch area in m^2.
    double slipRatio { 0.0 }; ///< Longitudinal slip ratio kappa.
    double slipAngle { 0.0 }; ///< Lateral slip angle alpha in radians.
    double tractiveForce { 0.0 }; ///< Longitudinal tractive or braking force in Newtons.
    double corneringForce { 0.0 }; ///< Lateral cornering force in Newtons.
    bool isWheelLifted { false }; ///< True when wheel has unweighted and detached from ground.
};

/// @struct QuadBikeTelemetry
/// @brief Diagnostic telemetry snapshot for QuadBikeDynamics.
struct QuadBikeTelemetry {
    double forwardSpeed { 0.0 }; ///< Forward longitudinal velocity in m/s.
    double lateralSpeed { 0.0 }; ///< Lateral sideslip velocity in m/s.
    double yawRate { 0.0 }; ///< Chassis yaw rate in rad/s.

    math::Vec2 compositeCogOffset { 0.0, 0.0 }; ///< Dynamic composite COG offset from static COG in meters.
    double lateralRolloverIndex { 0.0 }; ///< Lateral rollover tipping index I_rollover.
    double longitudinalPitchoverIndex { 0.0 }; ///< Longitudinal pitchover tipping index I_pitchover.
    double rearAxleBindingTorque { 0.0 }; ///< Solid spool axle yaw scrub resisting torque in N*m.

    std::array<QuadWheelState, 4> wheels {}; ///< Telemetry for FL (0), FR (1), RL (2), RR (3).

    bool isTwoWheelSideRiding { false }; ///< True when both tires on one side have lifted off ground.
    bool isWheelie { false }; ///< True when front wheels are unweighted and lifted off ground.
    bool isStoppie { false }; ///< True when rear wheels are unweighted and lifted off ground.
    bool isAirborne { false }; ///< True when all four tires are completely airborne.
};

/// @class QuadBikeDynamics
/// @brief 4-wheel all-terrain vehicle (ATV) dynamics coordinator.
/// @details Simulates low-pressure balloon tire compliance and contact footprint expansion,
/// solid swingarm rear drive axle with spool scrub or open differential, independent double-wishbone
/// front suspension, active 2D rider torso mass displacement, and multi-axis rollover margins.
class QuadBikeDynamics final : public IVehicleDynamics {
public:
    /// @brief Default constructor using standard all-terrain quad bike configuration.
    QuadBikeDynamics() noexcept;

    /// @brief Parameterized constructor using custom quad bike configuration.
    /// @param[in] config Physical, tire, and suspension parameters.
    explicit QuadBikeDynamics(const QuadBikeConfig& config) noexcept;

    /// @brief Advances simulation step, applying tire forces and solid axle torques.
    /// @param[in,out] body Target rigid body representing quad bike chassis.
    /// @param[in] dt Timestep duration in seconds.
    void step(physics::RigidBody2D& body, double dt) noexcept override;

    /// @brief Resets transient dynamics, slip states, and tire deflections.
    void reset() noexcept override;

    /// @brief Queries whether quad bike is airborne.
    /// @return True if all four wheels have lost ground contact.
    [[nodiscard]] bool isAirborne() const noexcept override;

    /// @brief Gets active quad bike configuration.
    /// @return Const reference to QuadBikeConfig.
    [[nodiscard]] const QuadBikeConfig& getConfig() const noexcept;

    /// @brief Updates active quad bike configuration.
    /// @param[in] config New configuration parameters.
    void setConfig(const QuadBikeConfig& config) noexcept;

    /// @brief Gets current rider control inputs.
    /// @return Const reference to QuadBikeInput.
    [[nodiscard]] const QuadBikeInput& getInput() const noexcept;

    /// @brief Sets current rider control inputs.
    /// @param[in] input New rider commands.
    void setInput(const QuadBikeInput& input) noexcept;

    /// @brief Gets current diagnostic telemetry.
    /// @return Const reference to QuadBikeTelemetry.
    [[nodiscard]] const QuadBikeTelemetry& getTelemetry() const noexcept;

private:
    QuadBikeConfig m_config {};
    QuadBikeInput m_input {};
    QuadBikeTelemetry m_telemetry {};

    double m_lastForwardSpeed { 0.0 };
    bool m_isAirborne { false };
};

} // namespace vehicle_sim::dynamics
