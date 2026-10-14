#pragma once

#include <cmath>

#include "libs/VehicleSimCore/dynamics/IVehicleDynamics.h"
#include "libs/VehicleSimCore/math/Vec2.h"

namespace vehicle_sim::physics {
class RigidBody2D;
} // namespace vehicle_sim::physics

namespace vehicle_sim::dynamics {

/// @struct SnowmobileConfig
/// @brief Physical, ski carving, track propulsion, and snowpack parameters for SnowmobileDynamics.
struct SnowmobileConfig {
    double chassisMass { 230.0 }; ///< Dry mass of snowmobile chassis and engine in kg.
    double riderMass { 80.0 }; ///< Active rider body mass in kg.
    double rollInertia { 42.0 }; ///< Moment of inertia about longitudinal roll axis in kg*m^2.
    double yawInertia { 120.0 }; ///< Moment of inertia about vertical yaw axis in kg*m^2.

    double distFrontSkis { 0.95 }; ///< Longitudinal distance from static COG to front ski pivot in meters.
    double distRearTrack { 0.65 }; ///< Longitudinal distance from static COG to track center in meters.
    double skiStanceWidth { 1.05 }; ///< Transverse distance between left and right ski centers in meters.
    double cogHeight { 0.48 }; ///< Height of composite COG above ground plane in meters.

    // Ski Carving Parameters
    double skiLength { 1.10 }; ///< Front ski keel length in meters.
    double skiWidth { 0.18 }; ///< Front ski base width in meters.
    double skiBaseFriction { 0.08 }; ///< Kinetic friction of polyethylene base on cold snow.
    double skiCarveStiffness { 1.45 }; ///< Carbide runner edge carving multiplier k_edge.
    double maxSteerAngle { 0.52 }; ///< Maximum handlebar steering angle in radians (~30 degrees).
    double snowShearStrength { 6000.0 }; ///< Snowpack cohesive shear strength tau_snow in N/m^2.

    // Track Propulsion & Snow Mechanics
    double trackLength { 1.40 }; ///< Ground contact length of rear continuous track in meters.
    double trackWidth { 0.40 }; ///< Rear track width in meters.
    double sprocketRadius { 0.12 }; ///< Drive sprocket radius in meters.
    double maxDriveForce { 3800.0 }; ///< Peak engine tractive force at drive sprocket in Newtons.
    double maxBrakeForce { 4200.0 }; ///< Hydraulic disc brake capacity on drive track shaft in Newtons.
    double trackFrictionPeak { 0.75 }; ///< Peak friction coefficient of paddle track on snow.
    double snowDensity { 280.0 }; ///< Loose snow density rho_snow in kg/m^3.
    double snowCompactionModulus { 18000.0 }; ///< Bekker snow sinkage stiffness k_snow in N/m^(n+2).
    double snowSinkageExponent { 1.15 }; ///< Bekker sinkage exponent n_snow.
    double slipTrenchingGain { 1.80 }; ///< Trenching sinkage multiplier under high slip ratio.

    // Suspension & Roll Dynamics
    double rollStiffness { 3200.0 }; ///< Front A-arm suspension anti-roll stiffness in N*m/rad.
    double rollDamping { 380.0 }; ///< Roll angular velocity damping coefficient in N*m*s/rad.
    double riderMaxShiftY { 0.45 }; ///< Maximum lateral displacement of rider Center of Mass in meters.
};

/// @struct SnowmobileInput
/// @brief Driver control commands for snowmobile dynamics.
struct SnowmobileInput {
    double throttle { 0.0 }; ///< Engine throttle command in range [0.0, 1.0].
    double brake { 0.0 }; ///< Track service brake command in range [0.0, 1.0].
    double steer { 0.0 }; ///< Handlebar steer command in range [-1.0, 1.0] (+1 right, -1 left).
    double riderLean { 0.0 }; ///< Active rider body lean command in range [-1.0, 1.0] (+1 right, -1 left).
};

/// @struct SnowmobileTelemetry
/// @brief Diagnostic telemetry snapshot for snowmobile dynamics.
struct SnowmobileTelemetry {
    double forwardSpeed { 0.0 }; ///< Forward longitudinal velocity in m/s.
    double lateralSpeed { 0.0 }; ///< Lateral sideslip velocity in m/s.
    double yawRate { 0.0 }; ///< Chassis yaw rate in rad/s.
    double rollAngle { 0.0 }; ///< Dynamic chassis roll angle theta_roll in radians.
    double rollRate { 0.0 }; ///< Chassis roll angular velocity in rad/s.

    double leftSkiLoad { 0.0 }; ///< Dynamic normal force on left ski in Newtons.
    double rightSkiLoad { 0.0 }; ///< Dynamic normal force on right ski in Newtons.
    double trackLoad { 0.0 }; ///< Dynamic normal force on rear continuous track in Newtons.

    double leftSkiCarveForce { 0.0 }; ///< Lateral edge carving force from left ski in Newtons.
    double rightSkiCarveForce { 0.0 }; ///< Lateral edge carving force from right ski in Newtons.
    double trackTractiveForce { 0.0 }; ///< Forward thrust from drive track paddles in Newtons.
    double trackTrenchingDrag { 0.0 }; ///< Opposing snow compaction & trenching resistance in Newtons.
    double trackSlipRatio { 0.0 }; ///< Longitudinal slip ratio kappa_track.
    double snowSinkageDepth { 0.0 }; ///< Snow sinkage / trench depth z_sink in meters.

    bool isInsideSkiLifted { false }; ///< True when inside ski unweights and lifts off snow.
    bool isPowerWheelie { false }; ///< True when front skis unweight completely under acceleration.
    bool isAirborne { false }; ///< True when chassis is completely detached from snow surface.
};

/// @class SnowmobileDynamics
/// @brief Dual-ski and continuous track winter powersport dynamics coordinator.
/// @details Simulates front steerable skis with snow friction and carbide edge carving,
/// continuous rear rubber track paddle propulsion with Bekker compaction trenching drag,
/// dynamic roll angle equilibrium, and active rider weight shifting.
class SnowmobileDynamics final : public IVehicleDynamics {
public:
    /// @brief Default constructor using standard mountain snowmobile configuration.
    SnowmobileDynamics() noexcept;

    /// @brief Parameterized constructor using custom snowmobile configuration.
    /// @param[in] config Physical and snow parameters.
    explicit SnowmobileDynamics(const SnowmobileConfig& config) noexcept;

    /// @brief Advances simulation step, applying ski and track forces to the rigid body.
    /// @param[in,out] body Target rigid body representing snowmobile chassis.
    /// @param[in] dt Timestep duration in seconds.
    void step(physics::RigidBody2D& body, double dt) noexcept override;

    /// @brief Resets transient dynamics states and roll angle.
    void reset() noexcept override;

    /// @brief Queries whether snowmobile is airborne.
    /// @return True if skis and track are completely off the snow surface.
    [[nodiscard]] bool isAirborne() const noexcept override;

    /// @brief Gets active snowmobile configuration.
    /// @return Const reference to SnowmobileConfig.
    [[nodiscard]] const SnowmobileConfig& getConfig() const noexcept;

    /// @brief Updates active snowmobile configuration.
    /// @param[in] config New configuration parameters.
    void setConfig(const SnowmobileConfig& config) noexcept;

    /// @brief Gets current driver control inputs.
    /// @return Const reference to SnowmobileInput.
    [[nodiscard]] const SnowmobileInput& getInput() const noexcept;

    /// @brief Sets current driver control inputs.
    /// @param[in] input New driver commands.
    void setInput(const SnowmobileInput& input) noexcept;

    /// @brief Gets current diagnostic telemetry.
    /// @return Const reference to SnowmobileTelemetry.
    [[nodiscard]] const SnowmobileTelemetry& getTelemetry() const noexcept;

private:
    SnowmobileConfig m_config{};
    SnowmobileInput m_input{};
    SnowmobileTelemetry m_telemetry{};

    double m_rollAngleRad{ 0.0 };
    double m_rollRateRadPerSec{ 0.0 };
    double m_lastForwardSpeed{ 0.0 };
    bool m_isAirborne{ false };
};

} // namespace vehicle_sim::dynamics
