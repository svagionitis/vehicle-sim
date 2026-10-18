#pragma once

#include "Joint2D.h"
#include "libs/VehicleSimCore/math/Mat2.h"
#include "libs/VehicleSimCore/math/Vec2.h"

namespace vehicle_sim::physics {

/// @enum LimitState
/// @brief Status of angular limit constraint engagement.
enum class LimitState {
    Inactive, ///< Relative angle is inside allowable range.
    AtLower, ///< Relative angle reached or exceeded lower limit.
    AtUpper, ///< Relative angle reached or exceeded upper limit.
    Equal ///< Lower and upper limits are identical (locked rotation).
};

/// @struct RevoluteJointConfig
/// @brief Configuration parameters for RevoluteJoint2D.
struct RevoluteJointConfig : public JointConfig {
    double referenceAngle { 0.0 }; ///< Relative body orientation offset in radians (thetaB - thetaA).
    bool enableLimit { false }; ///< True to enforce angular limits.
    double lowerAngle { 0.0 }; ///< Minimum allowed relative angle in radians.
    double upperAngle { 0.0 }; ///< Maximum allowed relative angle in radians.
    bool enableMotor { false }; ///< True to activate continuous angular motor.
    double motorSpeed { 0.0 }; ///< Commanded rotational velocity in rad/s.
    double maxMotorTorque { 0.0 }; ///< Peak motor torque capacity in N*m.
};

/// @class RevoluteJoint2D
/// @brief Pin/hinge joint constraining two bodies to rotate around a common world point.
/// @details Enforces 2-DOF translational coincidence between anchor points with an analytic 2x2
/// effective mass solver. Optionally enforces 1-DOF angular limits and active motor torque.
class RevoluteJoint2D final : public Joint2D {
public:
    /// @brief Parameterized constructor initializing revolute joint.
    /// @param[in] config Revolute joint configuration parameters.
    explicit RevoluteJoint2D(const RevoluteJointConfig& config) noexcept;

    /// @brief Computes relative joint angle in radians (thetaB - thetaA - referenceAngle).
    /// @return Relative angle in [-pi, pi) radians.
    [[nodiscard]] double getJointAngle() const noexcept;

    /// @brief Computes relative angular velocity in rad/s (omegaB - omegaA).
    /// @return Relative rotational speed in rad/s.
    [[nodiscard]] double getJointSpeed() const noexcept;

    /// @brief Queries whether angular limits are active.
    /// @return True if limits are enforced.
    [[nodiscard]] bool isLimitEnabled() const noexcept;

    /// @brief Enables or disables angular limits.
    /// @param[in] flag True to enable limits.
    void enableLimit(bool flag) noexcept;

    /// @brief Gets lower angular limit in radians.
    /// @return Lower angle in radians.
    [[nodiscard]] double getLowerLimit() const noexcept;

    /// @brief Gets upper angular limit in radians.
    /// @return Upper angle in radians.
    [[nodiscard]] double getUpperLimit() const noexcept;

    /// @brief Sets lower and upper angular limits in radians.
    /// @param[in] lower Lower limit in radians.
    /// @param[in] upper Upper limit in radians.
    void setLimits(double lower, double upper) noexcept;

    /// @brief Queries whether angular motor is active.
    /// @return True if motor is enabled.
    [[nodiscard]] bool isMotorEnabled() const noexcept;

    /// @brief Enables or disables angular motor.
    /// @param[in] flag True to enable motor.
    void enableMotor(bool flag) noexcept;

    /// @brief Gets commanded motor speed in rad/s.
    /// @return Target angular velocity in rad/s.
    [[nodiscard]] double getMotorSpeed() const noexcept;

    /// @brief Sets commanded motor speed in rad/s.
    /// @param[in] speedRadPerSec Target angular velocity in rad/s.
    void setMotorSpeed(double speedRadPerSec) noexcept;

    /// @brief Gets maximum motor torque capacity in N*m.
    /// @return Maximum motor torque.
    [[nodiscard]] double getMaxMotorTorque() const noexcept;

    /// @brief Sets maximum motor torque capacity in N*m.
    /// @param[in] torqueNm Maximum torque in N*m.
    void setMaxMotorTorque(double torqueNm) noexcept;

    /// @brief Computes current motor torque exerted on body B.
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
    double mReferenceAngle { 0.0 };
    bool mEnableLimit { false };
    double mLowerAngle { 0.0 };
    double mUpperAngle { 0.0 };
    bool mEnableMotor { false };
    double mMotorSpeed { 0.0 };
    double mMaxMotorTorque { 0.0 };

    // Solver caches
    math::Vec2 mArmA { 0.0, 0.0 };
    math::Vec2 mArmB { 0.0, 0.0 };
    math::Mat2 mEffectiveMass {};
    math::Vec2 mPointImpulse { 0.0, 0.0 };
    double mMotorImpulse { 0.0 };
    double mLimitImpulse { 0.0 };
    double mAngularMass { 0.0 };
    LimitState mLimitState { LimitState::Inactive };
    math::Vec2 mPointBias { 0.0, 0.0 };
    double mDt { 0.0 };
};

} // namespace vehicle_sim::physics
