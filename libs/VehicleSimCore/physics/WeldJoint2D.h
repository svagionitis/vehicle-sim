#pragma once

#include <functional>

#include "Joint2D.h"
#include "libs/VehicleSimCore/math/Mat2.h"
#include "libs/VehicleSimCore/math/Vec2.h"

namespace vehicle_sim::physics {

/// @struct WeldJointConfig
/// @brief Configuration parameters for WeldJoint2D.
struct WeldJointConfig : public JointConfig {
    double referenceAngle { 0.0 }; ///< Relative body orientation offset in radians (thetaB - thetaA).
    double frequencyHz { 0.0 }; ///< Soft compliance oscillation frequency in Hz (0 = rigid lock).
    double dampingRatio { 0.0 }; ///< Soft compliance damping ratio [0 = undamped, 1 = critical].
    double breakForce { 0.0 }; ///< Reaction force rupture threshold in Newtons (0 = unbreakable).
    double breakTorque { 0.0 }; ///< Reaction torque rupture threshold in N*m (0 = unbreakable).
};

/// @class WeldJoint2D
/// @brief Fixed 3-DOF structural bond locking 2 translational and 1 rotational DOFs.
/// @details Supports regularized elastic compliance and breakable structural rupture.
class WeldJoint2D final : public Joint2D {
public:
    /// @brief Parameterized constructor initializing weld joint.
    /// @param[in] config Weld joint configuration.
    explicit WeldJoint2D(const WeldJointConfig& config) noexcept;

    /// @brief Gets relative reference angle in radians.
    /// @return Reference angle in radians.
    [[nodiscard]] double getReferenceAngle() const noexcept;

    /// @brief Gets soft compliance frequency in Hz.
    /// @return Frequency in Hz.
    [[nodiscard]] double getFrequencyHz() const noexcept;

    /// @brief Sets soft compliance frequency in Hz.
    /// @param[in] hz Frequency in Hz (0 for rigid lock).
    void setFrequencyHz(double hz) noexcept;

    /// @brief Gets soft compliance damping ratio.
    /// @return Damping ratio.
    [[nodiscard]] double getDampingRatio() const noexcept;

    /// @brief Sets soft compliance damping ratio.
    /// @param[in] ratio Damping ratio.
    void setDampingRatio(double ratio) noexcept;

    /// @brief Queries whether the weld joint has ruptured.
    /// @return True if broken.
    [[nodiscard]] bool isBroken() const noexcept override;

    /// @brief Gets reaction force rupture threshold in Newtons.
    /// @return Break force in N (0 = unbreakable).
    [[nodiscard]] double getBreakForce() const noexcept;

    /// @brief Sets reaction force rupture threshold in Newtons.
    /// @param[in] force Break force in N (0 = unbreakable).
    void setBreakForce(double force) noexcept;

    /// @brief Gets reaction torque rupture threshold in N*m.
    /// @return Break torque in N*m (0 = unbreakable).
    [[nodiscard]] double getBreakTorque() const noexcept;

    /// @brief Sets reaction torque rupture threshold in N*m.
    /// @param[in] torque Break torque in N*m (0 = unbreakable).
    void setBreakTorque(double torque) noexcept;

    /// @brief Registers a callback invoked upon structural rupture.
    /// @param[in] callback Callable receiving reference to ruptured joint.
    void setOnBreakCallback(std::function<void(WeldJoint2D&)> callback) noexcept;

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
    double mReferenceAngle { 0.0 };
    double mFrequencyHz { 0.0 };
    double mDampingRatio { 0.0 };
    double mBreakForce { 0.0 };
    double mBreakTorque { 0.0 };
    bool mBroken { false };
    std::function<void(WeldJoint2D&)> mOnBreakCallback {};

    // Solver caches
    math::Vec2 mArmA { 0.0, 0.0 };
    math::Vec2 mArmB { 0.0, 0.0 };
    math::Mat2 mMassK11 {};
    math::Vec2 mMassK12 { 0.0, 0.0 };
    double mMassK22 { 0.0 };
    math::Vec2 mLinearImpulse { 0.0, 0.0 };
    double mAngularImpulse { 0.0 };
    double mBias { 0.0 };
    double mGamma { 0.0 };
    double mDt { 0.0 };
};

} // namespace vehicle_sim::physics
