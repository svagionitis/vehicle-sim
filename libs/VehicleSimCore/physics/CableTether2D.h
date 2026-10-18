#pragma once

#include <cstddef>
#include <functional>
#include <vector>

#include "Joint2D.h"
#include "libs/VehicleSimCore/math/Vec2.h"

namespace vehicle_sim::physics {

/// @enum WinchMode
/// @brief Operational state of the dynamic winch spool drum.
enum class WinchMode {
    Locked, ///< Drum brake engaged; rest length remains fixed.
    HaulIn, ///< Powered reeling in; decreases rest length up to motor pull limit.
    Payout, ///< Powered unreeling; increases rest length at controlled speed.
    FreeWheel ///< Drum clutch disengaged; cable unspools freely under external tension.
};

/// @struct CableTetherConfig
/// @brief Configuration initialization parameters for CableTether2D.
struct CableTetherConfig : public JointConfig {
    double restLength { -1.0 }; ///< Initial rest length in meters. If < 0, auto-computed from anchors.
    double minRestLength { 0.2 }; ///< Minimum allowable reeled-in length in meters.
    double maxRestLength { 50.0 }; ///< Maximum drum cable payout capacity in meters.
    double frequencyHz { 0.0 }; ///< Soft compliance frequency in Hz (0 = rigid inextensible cable).
    double dampingRatio { 0.0 }; ///< Soft compliance damping ratio [0 = undamped, 1 = critical].
    double breakTension { 0.0 }; ///< Ultimate tensile failure threshold in Newtons (0 = unbreakable).
    double maxPullForce { 20000.0 }; ///< Peak winch motor stall pull force in Newtons.
    double spoolSpeed { 1.0 }; ///< Commanded winch spool rate in m/s (positive = payout, negative = haul).
    WinchMode winchMode { WinchMode::Locked }; ///< Initial winch operating mode.
};

/// @class CableTether2D
/// @brief Unilateral tensile cable, winch line, and elastic tether constraint.
/// @details Enforces tensile forces exclusively (no compressive resistance), supports dynamic
/// motor spooling (haul-in / payout / freewheel), structural snap rupture, static world anchors,
/// and analytical catenary sag evaluation.
class CableTether2D final : public Joint2D {
public:
    /// @brief Parameterized constructor initializing cable tether.
    /// @param[in] config Cable and winch configuration parameters.
    explicit CableTether2D(const CableTetherConfig& config) noexcept;

    /// @brief Gets the current unstretched rest length.
    /// @return Rest length in meters.
    [[nodiscard]] double getRestLength() const noexcept;

    /// @brief Sets the unstretched rest length directly.
    /// @param[in] length New rest length in meters.
    void setRestLength(double length) noexcept;

    /// @brief Gets current separation distance between anchors.
    /// @return Current Euclidean distance in meters.
    [[nodiscard]] double getCurrentLength() const noexcept;

    /// @brief Gets minimum allowable rest length limit.
    /// @return Minimum length in meters.
    [[nodiscard]] double getMinRestLength() const noexcept;

    /// @brief Sets minimum allowable rest length limit.
    /// @param[in] length Minimum length in meters.
    void setMinRestLength(double length) noexcept;

    /// @brief Gets maximum allowable rest length capacity.
    /// @return Maximum length in meters.
    [[nodiscard]] double getMaxRestLength() const noexcept;

    /// @brief Sets maximum allowable rest length capacity.
    /// @param[in] length Maximum length in meters.
    void setMaxRestLength(double length) noexcept;

    /// @brief Queries whether the cable is currently slack (distance <= rest length).
    /// @return True if cable is slack with zero tension.
    [[nodiscard]] bool isSlack() const noexcept;

    /// @brief Queries whether the cable is currently taut (distance > rest length).
    /// @return True if cable is taut under tension.
    [[nodiscard]] bool isTaut() const noexcept;

    /// @brief Queries whether the cable has ruptured/snapped.
    /// @return True if broken.
    [[nodiscard]] bool isBroken() const noexcept override;

    /// @brief Gets ultimate break tension threshold.
    /// @return Break tension in Newtons (0 = unbreakable).
    [[nodiscard]] double getBreakTension() const noexcept;

    /// @brief Sets ultimate break tension threshold.
    /// @param[in] tension Break tension in Newtons (0 = unbreakable).
    void setBreakTension(double tension) noexcept;

    /// @brief Gets current tension force exerted by the cable.
    /// @return Tension force in Newtons.
    [[nodiscard]] double getTension() const noexcept;

    /// @brief Gets current cable strain ((d - L0) / L0).
    /// @return Engineering strain scalar (0 if slack).
    [[nodiscard]] double getStrain() const noexcept;

    /// @brief Gets stored elastic strain energy (0.5 * k * deltaL^2).
    /// @return Strain energy in Joules.
    [[nodiscard]] double getStrainEnergy() const noexcept;

    /// @brief Gets active winch spool mode.
    /// @return Current WinchMode enum.
    [[nodiscard]] WinchMode getWinchMode() const noexcept;

    /// @brief Sets active winch spool mode.
    /// @param[in] mode Desired WinchMode.
    void setWinchMode(WinchMode mode) noexcept;

    /// @brief Gets commanded winch spool speed.
    /// @return Spool speed in m/s.
    [[nodiscard]] double getSpoolSpeed() const noexcept;

    /// @brief Sets commanded winch spool speed.
    /// @param[in] speed Commanded spool speed in m/s.
    void setSpoolSpeed(double speed) noexcept;

    /// @brief Gets winch peak motor stall pull force.
    /// @return Max pull force in Newtons.
    [[nodiscard]] double getMaxPullForce() const noexcept;

    /// @brief Sets winch peak motor stall pull force.
    /// @param[in] force Max pull force in Newtons.
    void setMaxPullForce(double force) noexcept;

    /// @brief Gets soft compliance frequency in Hz.
    /// @return Frequency in Hz (0 = rigid inextensible).
    [[nodiscard]] double getFrequencyHz() const noexcept;

    /// @brief Sets soft compliance frequency in Hz.
    /// @param[in] hz Frequency in Hz.
    void setFrequencyHz(double hz) noexcept;

    /// @brief Gets soft compliance damping ratio.
    /// @return Damping ratio.
    [[nodiscard]] double getDampingRatio() const noexcept;

    /// @brief Sets soft compliance damping ratio.
    /// @param[in] ratio Damping ratio.
    void setDampingRatio(double ratio) noexcept;

    /// @brief Registers a callback invoked synchronously upon structural snap rupture.
    /// @param[in] callback Callable receiving reference to ruptured cable and failure tension.
    void setOnSnapCallback(std::function<void(CableTether2D&, double)> callback) noexcept;

    /// @brief Samples 2D parabolic catenary sag curve points between anchors.
    /// @param[in] numPoints Number of sample segments along span (minimum 2).
    /// @param[in] gravity Directional gravity vector in m/s^2.
    /// @param[out] outPoints Destination buffer receiving world coordinates.
    void sampleCatenary(std::size_t numPoints,
                        const math::Vec2& gravity,
                        std::vector<math::Vec2>& outPoints) const;

    /// @brief Pre-calculates effective mass, updates winch spooling, and warm-starts tension.
    /// @param[in] dt Simulation timestep in seconds.
    void initVelocityConstraints(double dt) noexcept override;

    /// @brief Solves unilateral velocity-level constraint equations using sequential impulses.
    void solveVelocityConstraints() noexcept override;

    /// @brief Solves position-level geometric drift to eliminate over-extension.
    /// @return True if position error is within tolerance or cable is slack.
    [[nodiscard]] bool solvePositionConstraints() noexcept override;

    /// @brief Gets reaction force vector exerted on body B in world coordinates.
    /// @param[in] invDt Inverse timestep (1.0 / dt).
    /// @return Reaction force in Newtons.
    [[nodiscard]] math::Vec2 getReactionForce(double invDt) const noexcept override;

    /// @brief Gets reaction torque scalar exerted on body B.
    /// @param[in] invDt Inverse timestep (1.0 / dt).
    /// @return Reaction torque in Newton-meters.
    [[nodiscard]] double getReactionTorque(double invDt) const noexcept override;

    /// @brief Captures warm-starting impulses, spool length, and rupture state for snapshots.
    /// @return JointSnapshotData with cached impulses.
    [[nodiscard]] JointSnapshotData captureState() const noexcept override;

    /// @brief Restores warm-starting impulses, spool length, and rupture state from snapshot.
    /// @param[in] state Cached impulses and rupture state.
    void restoreState(const JointSnapshotData& state) noexcept override;

private:
    void updateWinchSpool(double dt) noexcept;

    double mRestLength { 1.0 };
    double mMinRestLength { 0.2 };
    double mMaxRestLength { 50.0 };
    double mFrequencyHz { 0.0 };
    double mDampingRatio { 0.0 };
    double mBreakTension { 0.0 };
    double mMaxPullForce { 20000.0 };
    double mSpoolSpeed { 1.0 };
    WinchMode mWinchMode { WinchMode::Locked };
    bool mBroken { false };
    std::function<void(CableTether2D&, double)> mOnSnapCallback {};

    // Solver caches
    math::Vec2 mUnitSeparation { 1.0, 0.0 };
    math::Vec2 mArmA { 0.0, 0.0 };
    math::Vec2 mArmB { 0.0, 0.0 };
    double mEffectiveMass { 0.0 };
    double mImpulse { 0.0 };
    double mBias { 0.0 };
    double mGamma { 0.0 };
    double mCurrentTension { 0.0 };
    double mDt { 0.016 };
};

} // namespace vehicle_sim::physics
