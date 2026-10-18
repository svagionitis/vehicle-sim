#pragma once

namespace vehicle_sim::dynamics {

/// @struct ClutchConfig
/// @brief Mechanical friction and inertia parameters for clutch assembly.
struct ClutchConfig {
    double maxClutchTorque { 750.0 }; ///< Maximum static friction holding capacity in N*m.
    double slipThreshold { 1.5 }; ///< Angular velocity difference threshold for lockup in rad/s.
    double engagementRate { 10.0 }; ///< Clutch pedal actuation slew rate limit in 1/s.
};

/// @struct ClutchState
/// @brief Dynamic instantaneous metrics from the clutch interface.
struct ClutchState {
    double transferredTorque { 0.0 }; ///< Net torque transmitted to transmission input in N*m.
    double slipVelocity { 0.0 }; ///< Angular velocity difference (w_engine - w_trans) in rad/s.
    double clutchPedal { 0.0 }; ///< Filtered clutch pedal position [0 = engaged, 1 = disengaged].
    double lockRatio { 0.0 }; ///< Clamping engagement factor in [0, 1].
    bool isLocked { false }; ///< True if clutch plates are statically locked together.
    bool isSlipping { false }; ///< True if clutch plates are dynamically slipping.
};

/// @class FlywheelClutch2D
/// @brief Models flywheel rotational dynamics and Coulomb dry friction clutch.
class FlywheelClutch2D {
public:
    /// @brief Default constructor using baseline sports clutch configuration.
    FlywheelClutch2D() noexcept;

    /// @brief Parameterized constructor using custom clutch configuration.
    /// @param[in] config Friction capacity and threshold parameters.
    explicit FlywheelClutch2D(const ClutchConfig& config) noexcept;

    /// @brief Advances flywheel and clutch solver.
    /// @param[in] engineTorque Net torque delivered by prime mover in N*m.
    /// @param[in] loadTorque Reflected resistance torque from gearbox input in N*m.
    /// @param[in] engineInertia Flywheel/engine rotational inertia in kg*m^2.
    /// @param[in] transInertia Gearbox input reflected inertia in kg*m^2.
    /// @param[in] pedalCommand Driver clutch pedal input [0 = engaged, 1 = disengaged].
    /// @param[in] dt Timestep duration in seconds.
    /// @return Evaluated clutch output metrics.
    [[nodiscard]] ClutchState step(double engineTorque,
                                   double loadTorque,
                                   double engineInertia,
                                   double transInertia,
                                   double pedalCommand,
                                   double dt) noexcept;

    /// @brief Resets clutch state and rotational speeds.
    /// @param[in] initialEngineRpm Initial engine RPM.
    void reset(double initialEngineRpm = 900.0) noexcept;

    /// @brief Queries instantaneous engine angular velocity.
    /// @return Crankshaft speed in rad/s.
    [[nodiscard]] double getEngineSpeed() const noexcept;

    /// @brief Queries instantaneous transmission input angular velocity.
    /// @return Transmission input speed in rad/s.
    [[nodiscard]] double getTransSpeed() const noexcept;

    /// @brief Forces transmission input speed (e.g. from direct wheel lock).
    /// @param[in] speed New transmission input speed in rad/s.
    void setTransSpeed(double speed) noexcept;

    /// @brief Queries cached state from the last step.
    /// @return Most recent ClutchState snapshot.
    [[nodiscard]] const ClutchState& getState() const noexcept;

private:
    ClutchConfig config_ {};
    double engineAngVel_ { 94.24777960769379 }; ///< Initial 900 RPM.
    double transAngVel_ { 0.0 };
    double currentPedal_ { 0.0 };
    ClutchState lastState_ {};
};

} // namespace vehicle_sim::dynamics
