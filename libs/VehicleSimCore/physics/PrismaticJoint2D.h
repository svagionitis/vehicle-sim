#pragma once

#include "Joint2D.h"
#include "libs/VehicleSimCore/math/Mat2.h"
#include "libs/VehicleSimCore/math/Vec2.h"

namespace vehicle_sim::physics {

/// @struct PrismaticJointConfig
/// @brief Configuration parameters for PrismaticJoint2D.
struct PrismaticJointConfig : public JointConfig {
    math::Vec2 localAxisA { 1.0, 0.0 }; ///< Linear translation axis in body A local coordinates.
    double referenceAngle { 0.0 }; ///< Target relative body orientation offset (thetaB - thetaA).
    bool enableLimit { false }; ///< True to enforce translation limits.
    double lowerTranslation { 0.0 }; ///< Minimum allowable translation in meters along axis.
    double upperTranslation { 0.0 }; ///< Maximum allowable translation in meters along axis.
    bool enableMotor { false }; ///< True to activate continuous linear motor.
    double motorSpeed { 0.0 }; ///< Target linear velocity in m/s along axis.
    double maxMotorForce { 0.0 }; ///< Peak motor push/pull force capacity in Newtons.
};

/// @class PrismaticJoint2D
/// @brief Linear slider joint constraining relative rotation and lateral translation.
/// @details Allows 1-DOF linear motion along a specified axis anchored in body A.
class PrismaticJoint2D final : public Joint2D {
public:
    /// @brief Parameterized constructor initializing prismatic joint.
    /// @param[in] config Prismatic joint configuration.
    explicit PrismaticJoint2D(const PrismaticJointConfig& config) noexcept;

    /// @brief Computes current linear translation along joint axis in meters.
    /// @return Current translation in meters.
    [[nodiscard]] double getJointTranslation() const noexcept;

    /// @brief Computes current linear relative velocity along joint axis in m/s.
    /// @return Current relative speed in m/s.
    [[nodiscard]] double getJointSpeed() const noexcept;

    /// @brief Queries whether translation limits are active.
    /// @return True if limits are enforced.
    [[nodiscard]] bool isLimitEnabled() const noexcept;

    /// @brief Enables or disables translation limits.
    /// @param[in] flag True to enable limits.
    void enableLimit(bool flag) noexcept;

    /// @brief Gets lower translation limit in meters.
    /// @return Lower limit in meters.
    [[nodiscard]] double getLowerLimit() const noexcept;

    /// @brief Gets upper translation limit in meters.
    /// @return Upper limit in meters.
    [[nodiscard]] double getUpperLimit() const noexcept;

    /// @brief Sets lower and upper translation limits in meters.
    /// @param[in] lower Lower limit in meters.
    /// @param[in] upper Upper limit in meters.
    void setLimits(double lower, double upper) noexcept;

    /// @brief Queries whether linear motor is active.
    /// @return True if motor is enabled.
    [[nodiscard]] bool isMotorEnabled() const noexcept;

    /// @brief Enables or disables linear motor.
    /// @param[in] flag True to enable motor.
    void enableMotor(bool flag) noexcept;

    /// @brief Gets commanded motor speed in m/s.
    /// @return Target speed in m/s.
    [[nodiscard]] double getMotorSpeed() const noexcept;

    /// @brief Sets commanded motor speed in m/s.
    /// @param[in] speed Target speed in m/s.
    void setMotorSpeed(double speed) noexcept;

    /// @brief Gets maximum motor force capacity in Newtons.
    /// @return Maximum force in N.
    [[nodiscard]] double getMaxMotorForce() const noexcept;

    /// @brief Sets maximum motor force capacity in Newtons.
    /// @param[in] force Maximum force in N.
    void setMaxMotorForce(double force) noexcept;

    /// @brief Computes current motor force exerted along axis in Newtons.
    /// @param[in] invDt Inverse simulation timestep (1.0 / dt).
    /// @return Motor force in N.
    [[nodiscard]] double getMotorForce(double invDt) const noexcept;

    /// @brief Pre-calculates effective mass, Baumgarte bias, and applies warm-starting impulses.
    /// @param[in] dt Simulation timestep in seconds.
    void initVelocityConstraints(double dt) noexcept override;

    /// @brief Solves velocity-level constraint equations using sequential impulses.
    void solveVelocityConstraints() noexcept override;

    /// @brief Solves position-level geometric drift to eliminate anchor separation and angle drift.
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
    math::Vec2 mLocalAxisA { 1.0, 0.0 };
    double mReferenceAngle { 0.0 };
    bool mEnableLimit { false };
    double mLowerTranslation { 0.0 };
    double mUpperTranslation { 0.0 };
    bool mEnableMotor { false };
    double mMotorSpeed { 0.0 };
    double mMaxMotorForce { 0.0 };

    // Solver caches
    math::Vec2 mAxis { 1.0, 0.0 };
    math::Vec2 mPerpAxis { 0.0, 1.0 };
    math::Vec2 mArmA { 0.0, 0.0 };
    math::Vec2 mArmB { 0.0, 0.0 };
    math::Vec2 mDelta { 0.0, 0.0 };
    double mArmA1 { 0.0 };
    double mArmB1 { 0.0 };
    double mArmA2 { 0.0 };
    double mArmB2 { 0.0 };
    math::Mat2 mEffectiveMass {};
    double mAxialMass { 0.0 };
    math::Vec2 mImpulse { 0.0, 0.0 };
    double mMotorImpulse { 0.0 };
    double mLimitImpulse { 0.0 };
    double mDt { 0.0 };
};

} // namespace vehicle_sim::physics
