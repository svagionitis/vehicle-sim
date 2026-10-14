#pragma once

#include <cmath>
#include <cstddef>

#include "libs/VehicleSimCore/dynamics/IVehicleDynamics.h"
#include "libs/VehicleSimCore/math/Spline2D.h"
#include "libs/VehicleSimCore/math/Vec2.h"

namespace vehicle_sim::physics {
class RigidBody2D;
} // namespace vehicle_sim::physics

namespace vehicle_sim::dynamics {

/// @struct TrainRailConfig
/// @brief Physical configuration parameters for rail vehicle dynamics.
/// @details Defines locomotive/car mass, wheelset geometry, conical wheel taper,
/// flange clearance, Kalker creep coefficients, draft gear spring-damping, and adhesion limits.
struct TrainRailConfig {
    double mass { 80000.0 }; ///< Locomotive/car total mass in kg (~80 tonnes).
    double momentOfInertia { 1200000.0 }; ///< Yaw moment of inertia in kg*m^2.
    double carLength { 20.0 }; ///< Longitudinal coupler-to-coupler length in meters.
    double trackGauge { 1.435 }; ///< Standard track rail gauge (2b) in meters.
    double wheelRadius { 0.457 }; ///< Nominal wheel rolling radius (r0) in meters.
    double wheelConicity { 0.05 }; ///< Conical wheel tread taper gamma (e.g. 1/20 = 0.05).
    double flangeClearance { 0.010 }; ///< Lateral clearance gap delta_gap before flange contact in meters (~10 mm).
    double flangeStiffness { 1.5e7 }; ///< Flange contact strike spring stiffness in N/m.
    double flangeDamping { 2.5e5 }; ///< Flange strike viscous damping in N*s/m.
    double flangeAngle { 1.134 }; ///< Wheel flange contact face angle in radians (~65 degrees).
    double flangeFriction { 0.30 }; ///< Wheel flange-to-rail steel friction coefficient.

    double kalkerCoefficient { 1.2e7 }; ///< Kalker longitudinal creep coefficient f11 in Newtons.
    double dryAdhesion { 0.32 }; ///< Clean dry rail friction coefficient.
    double wetAdhesion { 0.15 }; ///< Wet / contaminated rail friction coefficient.
    double sandedAdhesion { 0.40 }; ///< Maximum adhesion coefficient with active sand injection.
    double maxTractiveForce { 300000.0 }; ///< Peak traction propulsion force limit in Newtons (~300 kN).
    double maxBrakingForce { 250000.0 }; ///< Maximum pneumatic service braking force in Newtons.

    double draftSlack { 0.035 }; ///< Free coupler deadband slack space delta_slack in meters (+/- 35 mm).
    double draftStiffness { 4.0e6 }; ///< Draft gear cushioning spring stiffness in N/m.
    double draftDamping { 1.8e5 }; ///< Draft gear hydraulic / friction damping in N*s/m.
    double cantAngle { 0.0 }; ///< Static or baseline track superelevation angle in radians.
};

/// @struct TrainRailInput
/// @brief Operator control inputs for a rail locomotive.
struct TrainRailInput {
    double throttle { 0.0 }; ///< Commanded traction throttle in range [0.0, 1.0].
    double serviceBrake { 0.0 }; ///< Commanded pneumatic service brake in range [0.0, 1.0].
    double dynamicBrake { 0.0 }; ///< Commanded rheostatic/regenerative brake in range [0.0, 1.0].
    int reverser { 1 }; ///< Direction reverser: 1 for forward, 0 for neutral, -1 for reverse.
    bool sanderActive { false }; ///< Sander system toggle injecting traction sand onto rails.
};

/// @struct TrainRailTelemetry
/// @brief Diagnostic telemetry snapshot for rail vehicle simulation.
struct TrainRailTelemetry {
    double trackArcLength { 0.0 }; ///< Current longitudinal arc-length s along track in meters.
    double trackSpeed { 0.0 }; ///< Forward longitudinal velocity s_dot in m/s.
    double trackCurvature { 0.0 }; ///< Track curvature kappa(s) at current position in 1/m.
    double cantAngle { 0.0 }; ///< Local track cant / superelevation angle in radians.
    double uncompLateralAcc { 0.0 }; ///< Uncompensated lateral acceleration in m/s^2.
    double lateralDisplacement { 0.0 }; ///< Lateral wheelset deviation y from track centerline in meters.
    double huntingFrequency { 0.0 }; ///< Klingel kinematic hunting frequency in Hz.
    double flangeForce { 0.0 }; ///< Current lateral wheel flange strike force in Newtons.
    double nadalRatio { 0.0 }; ///< Current Nadal derailment criterion ratio Y/Q.
    double nadalLimit { 0.85 }; ///< Critical Nadal safety threshold limit.
    double creepRatio { 0.0 }; ///< Normalized longitudinal wheel-rail creepage xi_x.
    double tractiveForce { 0.0 }; ///< Applied longitudinal wheel-rail traction force in Newtons.
    double couplerForce { 0.0 }; ///< Transmitted coupler tension (+) or compression (-) in Newtons.
    bool isDerailmentRisk { false }; ///< True if Nadal ratio exceeds safe limit or excessive lateral lift.
    bool isSanding { false }; ///< True when sander system is actively augmenting adhesion.
};

/// @class TrainRailDynamics
/// @brief Guided track railway dynamics coordinator constrained to Spline2D tracks.
/// @details Implements 1D curvilinear arc-length kinematics along Spline2D, conical wheelset
/// Klingel hunting oscillations, flange contact reaction with Nadal derailment criteria,
/// Kalker linear creepage with Polach adhesion saturation, sander injection, and multi-car coupler slack action.
class TrainRailDynamics final : public IVehicleDynamics {
public:
    /// @brief Default constructor using baseline locomotive configuration.
    TrainRailDynamics() noexcept;

    /// @brief Parameterized constructor with custom railway configuration.
    /// @param[in] config Railway physical parameters and limits.
    explicit TrainRailDynamics(const TrainRailConfig& config) noexcept;

    /// @brief Advances simulation step, applying forces along track.
    /// @param[in,out] body Target rigid body representing rail vehicle chassis.
    /// @param[in] dt Timestep duration in seconds.
    void step(physics::RigidBody2D& body, double dt) noexcept override;

    /// @brief Resets transient control inputs, hunting states, and coupler slack.
    void reset() noexcept override;

    /// @brief Queries whether the rail vehicle is airborne or derailed.
    /// @return True if vehicle is detached from rails or tipped over.
    [[nodiscard]] bool isAirborne() const noexcept override;

    /// @brief Gets active railway configuration.
    /// @return Const reference to active TrainRailConfig.
    [[nodiscard]] const TrainRailConfig& getConfig() const noexcept;

    /// @brief Updates active railway configuration.
    /// @param[in] config New configuration parameters.
    void setConfig(const TrainRailConfig& config) noexcept;

    /// @brief Gets current operator control inputs.
    /// @return Const reference to active TrainRailInput.
    [[nodiscard]] const TrainRailInput& getInput() const noexcept;

    /// @brief Sets current operator control inputs.
    /// @param[in] input New driver commands.
    void setInput(const TrainRailInput& input) noexcept;

    /// @brief Gets current diagnostic telemetry.
    /// @return Const reference to active TrainRailTelemetry.
    [[nodiscard]] const TrainRailTelemetry& getTelemetry() const noexcept;

    /// @brief Attaches the parametric railway track spline.
    /// @param[in] track Pointer to parametric Spline2D track (non-owning).
    void setTrack(const math::Spline2D* track) noexcept;

    /// @brief Sets initial arc-length position on the track.
    /// @param[in] s Arc-length coordinate from track start in meters.
    void setArcLength(double s) noexcept;

    /// @brief Sets track longitudinal velocity in m/s.
    /// @param[in] speed Longitudinal velocity along track in m/s.
    void setTrackSpeed(double speed) noexcept;

    /// @brief Sets lateral displacement from track centerline in meters.
    /// @param[in] offset Lateral deviation in meters.
    void setLateralOffset(double offset) noexcept;

    /// @brief Sets the preceding linked car for coupler slack action.
    /// @param[in] leadCar Pointer to leading rail car dynamics (non-owning).
    void setLeadCar(const TrainRailDynamics* leadCar) noexcept;

    /// @brief Sets local track cant / superelevation angle.
    /// @param[in] cantRadians Superelevation angle in radians.
    void setCantAngle(double cantRadians) noexcept;

    /// @brief Queries whether derailment threshold has been breached.
    /// @return True if Nadal criterion or overturning limit is exceeded.
    [[nodiscard]] bool isDerailed() const noexcept;

    /// @brief Retrieves the current longitudinal arc-length along track.
    /// @return Arc-length coordinate in meters.
    [[nodiscard]] double getArcLength() const noexcept;

    /// @brief Retrieves current track forward speed.
    /// @return Speed in m/s.
    [[nodiscard]] double getTrackSpeed() const noexcept;

    /// @brief Retrieves current lateral wheelset displacement.
    /// @return Displacement in meters.
    [[nodiscard]] double getLateralOffset() const noexcept;

private:
    TrainRailConfig m_config {};
    TrainRailInput m_input {};
    TrainRailTelemetry m_telemetry {};
    const math::Spline2D* m_track { nullptr };
    const TrainRailDynamics* m_leadCar { nullptr };

    double m_arcLength { 0.0 };
    double m_speed { 0.0 };
    double m_lateralOffset { 0.0 };
    double m_lateralVelocity { 0.0 };
    double m_cantAngle { 0.0 };
    double m_couplerForce { 0.0 };
    bool m_isDerailed { false };

    void updateKinematics(double dt) noexcept;
    void evaluateFlangeAndNadal(double dt) noexcept;
    [[nodiscard]] double evaluateTractionAndCreep() noexcept;
    void evaluateCouplerForces() noexcept;
};

} // namespace vehicle_sim::dynamics
