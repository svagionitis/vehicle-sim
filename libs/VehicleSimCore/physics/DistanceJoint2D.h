#pragma once

#include "Joint2D.h"
#include "libs/VehicleSimCore/math/Vec2.h"

namespace vehicle_sim::physics {

/// @struct DistanceJointConfig
/// @brief Configuration parameters for DistanceJoint2D.
struct DistanceJointConfig : public JointConfig {
    double targetDistance {
        -1.0
    }; ///< Target rest length in meters. If negative, auto-initialized from initial anchor distance.
    double minDistance { 0.0 }; ///< Minimum allowable distance in meters.
    double maxDistance { 0.0 }; ///< Maximum allowable distance in meters.
    double frequencyHz { 0.0 }; ///< Harmonic oscillation frequency for soft joints in Hz (0 = rigid constraint).
    double dampingRatio { 0.0 }; ///< Damping ratio for soft joints [0.0 = undamped, 1.0 = critically damped].
};

/// @class DistanceJoint2D
/// @brief Constrains the distance between two body anchor points to remain constant or elastic.
/// @details Supports rigid bilateral distance locking with Baumgarte stabilization
/// as well as regularized soft spring-damper compliance.
class DistanceJoint2D final : public Joint2D {
public:
    /// @brief Parameterized constructor initializing distance joint.
    /// @param[in] config Distance joint parameters.
    explicit DistanceJoint2D(const DistanceJointConfig& config) noexcept;

    /// @brief Gets target rest distance.
    /// @return Target distance in meters.
    [[nodiscard]] double getTargetDistance() const noexcept;

    /// @brief Sets target rest distance.
    /// @param[in] distance Target distance in meters.
    void setTargetDistance(double distance) noexcept;

    /// @brief Computes current Euclidean distance between world anchors.
    /// @return Separation distance in meters.
    [[nodiscard]] double getCurrentDistance() const noexcept;

    /// @brief Gets soft spring oscillation frequency.
    /// @return Frequency in Hz.
    [[nodiscard]] double getFrequencyHz() const noexcept;

    /// @brief Sets soft spring oscillation frequency.
    /// @param[in] hz Frequency in Hz (0 for rigid constraint).
    void setFrequencyHz(double hz) noexcept;

    /// @brief Gets soft spring damping ratio.
    /// @return Damping ratio.
    [[nodiscard]] double getDampingRatio() const noexcept;

    /// @brief Sets soft spring damping ratio.
    /// @param[in] ratio Damping ratio.
    void setDampingRatio(double ratio) noexcept;

    /// @brief Pre-calculates effective mass, Baumgarte bias, and applies warm-starting impulses.
    /// @param[in] dt Simulation timestep in seconds.
    void initVelocityConstraints(double dt) noexcept override;

    /// @brief Solves velocity-level constraint equations using sequential impulses.
    void solveVelocityConstraints() noexcept override;

    /// @brief Solves position-level geometric drift to eliminate anchor separation.
    /// @return True if position constraint error is within tolerance.
    [[nodiscard]] bool solvePositionConstraints() noexcept override;

    /// @brief Gets reaction force vector exerted on body B in world coordinates.
    /// @param[in] invDt Inverse timestep (1.0 / dt).
    /// @return Reaction force in Newtons.
    [[nodiscard]] math::Vec2 getReactionForce(double invDt) const noexcept override;

    /// @brief Gets reaction torque scalar exerted on body B.
    /// @param[in] invDt Inverse timestep (1.0 / dt).
    /// @return Always 0.0 for pure axial distance constraint.
    [[nodiscard]] double getReactionTorque(double invDt) const noexcept override;

private:
    double mTargetDistance { 1.0 };
    double mMinDistance { 0.0 };
    double mMaxDistance { 0.0 };
    double mFrequencyHz { 0.0 };
    double mDampingRatio { 0.0 };

    // Solver caches
    math::Vec2 mUnitSeparation { 1.0, 0.0 };
    math::Vec2 mArmA { 0.0, 0.0 };
    math::Vec2 mArmB { 0.0, 0.0 };
    double mEffectiveMass { 0.0 };
    double mImpulse { 0.0 };
    double mBias { 0.0 };
    double mGamma { 0.0 };
};

} // namespace vehicle_sim::physics
