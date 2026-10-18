#pragma once

#include "Joint2D.h"
#include "libs/VehicleSimCore/math/Vec2.h"

namespace vehicle_sim::physics {

/// @struct WheelJointConfig
/// @brief Configuration parameters for WheelJoint2D.
struct WheelJointConfig : public JointConfig {
    math::Vec2 localAxisA { 0.0, 1.0 }; ///< Suspension strut axis in chassis local coordinates.
    double frequencyHz { 4.0 }; ///< Suspension harmonic oscillation frequency in Hz.
    double dampingRatio { 0.7 }; ///< Suspension damping ratio [0 = undamped, 1 = critical].
    bool enableLimit { false }; ///< True to enforce strut bump/droop travel limits.
    double lowerTranslation { -0.1 }; ///< Minimum rebound / droop limit in meters.
    double upperTranslation { 0.1 }; ///< Maximum compression / bump limit in meters.
    bool enableMotor { false }; ///< True to activate wheel hub drive/brake motor.
    double motorSpeed { 0.0 }; ///< Target wheel hub rotational speed in rad/s.
    double maxMotorTorque { 0.0 }; ///< Maximum hub motor torque in N*m.
};

/// @class WheelJoint2D
/// @brief Suspension strut joint constraining lateral drift and providing soft axial compliance.
/// @details Allows 1-DOF linear suspension travel and 1-DOF free wheel hub rotation.
class WheelJoint2D final : public Joint2D {
public:
    /// @brief Parameterized constructor initializing wheel joint.
    /// @param[in] config Wheel joint configuration.
    explicit WheelJoint2D(const WheelJointConfig& config) noexcept;

    /// @brief Computes current linear suspension deflection in meters along strut axis.
    /// @return Current translation in meters.
    [[nodiscard]] double getJointTranslation() const noexcept;

    /// @brief Computes linear suspension velocity in m/s along strut axis.
    /// @return Relative speed along strut axis in m/s.
    [[nodiscard]] double getJointLinearSpeed() const noexcept;

    /// @brief Computes wheel hub rotational speed in rad/s.
    /// @return Wheel angular velocity in rad/s.
    [[nodiscard]] double getJointAngularSpeed() const noexcept;

    /// @brief Gets suspension oscillation frequency in Hz.
    /// @return Frequency in Hz.
    [[nodiscard]] double getFrequencyHz() const noexcept;

    /// @brief Sets suspension oscillation frequency in Hz.
    /// @param[in] hz Frequency in Hz.
    void setFrequencyHz(double hz) noexcept;

    /// @brief Gets suspension damping ratio.
    /// @return Damping ratio.
    [[nodiscard]] double getDampingRatio() const noexcept;

    /// @brief Sets suspension damping ratio.
    /// @param[in] ratio Damping ratio.
    void setDampingRatio(double ratio) noexcept;

    /// @brief Queries whether suspension travel limits are active.
    /// @return True if travel limits are enforced.
    [[nodiscard]] bool isLimitEnabled() const noexcept;

    /// @brief Enables or disables travel limits.
    /// @param[in] flag True to enable limits.
    void enableLimit(bool flag) noexcept;

    /// @brief Gets lower travel limit in meters.
    /// @return Lower limit in meters.
    [[nodiscard]] double getLowerLimit() const noexcept;

    /// @brief Gets upper travel limit in meters.
    /// @return Upper limit in meters.
    [[nodiscard]] double getUpperLimit() const noexcept;

    /// @brief Sets travel limits in meters.
    /// @param[in] lower Lower limit in meters.
    /// @param[in] upper Upper limit in meters.
    void setLimits(double lower, double upper) noexcept;

    /// @brief Queries whether wheel motor is active.
    /// @return True if hub motor is enabled.
    [[nodiscard]] bool isMotorEnabled() const noexcept;

    /// @brief Enables or disables wheel motor.
    /// @param[in] flag True to enable motor.
    void enableMotor(bool flag) noexcept;

    /// @brief Gets commanded motor speed in rad/s.
    /// @return Target angular speed in rad/s.
    [[nodiscard]] double getMotorSpeed() const noexcept;

    /// @brief Sets commanded motor speed in rad/s.
    /// @param[in] speed Target angular speed in rad/s.
    void setMotorSpeed(double speed) noexcept;

    /// @brief Gets maximum motor torque in N*m.
    /// @return Maximum torque.
    [[nodiscard]] double getMaxMotorTorque() const noexcept;

    /// @brief Sets maximum motor torque in N*m.
    /// @param[in] torque Maximum torque in N*m.
    void setMaxMotorTorque(double torque) noexcept;

    /// @brief Computes current motor torque exerted on wheel hub in N*m.
    /// @param[in] invDt Inverse simulation timestep (1.0 / dt).
    /// @return Motor torque in N*m.
    [[nodiscard]] double getMotorTorque(double invDt) const noexcept;

    /// @brief Pre-calculates effective mass, Baumgarte bias, and applies warm-starting impulses.
    /// @param[in] dt Simulation timestep in seconds.
    void initVelocityConstraints(double dt) noexcept override;

    /// @brief Solves velocity-level constraint equations using sequential impulses.
    void solveVelocityConstraints() noexcept override;

    /// @brief Solves position-level geometric drift to eliminate anchor separation.
    /// @return True if position constraint error is within tolerance.
    [[nodiscard]] bool solvePositionConstraints() noexcept override;

    /// @brief Captures warm-starting impulses and internal constraint state for snapshots.
    /// @return JointSnapshotData with cached impulses.
    [[nodiscard]] JointSnapshotData captureState() const noexcept override;

    /// @brief Restores warm-starting impulses and internal constraint state from snapshot.
    /// @param[in] state Cached impulses and rupture state.
    void restoreState(const JointSnapshotData& state) noexcept override;

    /// @brief Gets reaction force vector exerted on body B in world coordinates.
    /// @param[in] invDt Inverse timestep (1.0 / dt).
    /// @return Reaction force in Newtons.
    [[nodiscard]] math::Vec2 getReactionForce(double invDt) const noexcept override;

    /// @brief Gets reaction torque scalar exerted on body B.
    /// @param[in] invDt Inverse timestep (1.0 / dt).
    /// @return Reaction torque in Newton-meters.
    [[nodiscard]] double getReactionTorque(double invDt) const noexcept override;

private:
    math::Vec2 mLocalAxisA { 0.0, 1.0 };
    double mFrequencyHz { 4.0 };
    double mDampingRatio { 0.7 };
    bool mEnableLimit { false };
    double mLowerTranslation { -0.1 };
    double mUpperTranslation { 0.1 };
    bool mEnableMotor { false };
    double mMotorSpeed { 0.0 };
    double mMaxMotorTorque { 0.0 };

    // Solver caches
    math::Vec2 mAxis { 0.0, 1.0 };
    math::Vec2 mPerpAxis { -1.0, 0.0 };
    math::Vec2 mArmA { 0.0, 0.0 };
    math::Vec2 mArmB { 0.0, 0.0 };
    math::Vec2 mDelta { 0.0, 0.0 };
    double mArmA1 { 0.0 };
    double mArmB1 { 0.0 };
    double mArmA2 { 0.0 };
    double mArmB2 { 0.0 };
    double mPerpMass { 0.0 };
    double mAxialMass { 0.0 };
    double mMotorMass { 0.0 };
    double mSpringBias { 0.0 };
    double mGamma { 0.0 };
    double mPerpImpulse { 0.0 };
    double mAxialImpulse { 0.0 };
    double mMotorImpulse { 0.0 };
    double mDt { 0.0 };
};

} // namespace vehicle_sim::physics
