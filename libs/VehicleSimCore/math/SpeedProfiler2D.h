#pragma once

#include <vector>

#include "libs/VehicleSimCore/math/Spline2D.h"
#include "libs/VehicleSimCore/math/Vec2.h"

namespace vehicle_sim::math {

/// @struct SpeedProfileLimits
/// @brief Physical dynamic constraints for trajectory velocity profiling.
struct SpeedProfileLimits {
    double maxCruiseSpeed { 40.0 }; ///< Maximum cruising speed on straights in m/s.
    double maxLatAccel { 9.81 }; ///< Maximum lateral acceleration capacity in m/s^2 (mu * g).
    double maxLongAccel { 4.5 }; ///< Maximum longitudinal acceleration in m/s^2.
    double maxBrakingDecel { 8.0 }; ///< Maximum braking deceleration capacity in m/s^2.
    double sampleStep { 1.0 }; ///< Discretization arc-length step in meters.
};

/// @struct SpeedMilestone
/// @brief Discrete profile milestone point along a parameterized curve.
struct SpeedMilestone {
    double distance { 0.0 }; ///< Arc-length distance along curve in meters.
    Vec2 position { 0.0, 0.0 }; ///< World coordinates of milestone in meters.
    double curvature { 0.0 }; ///< Signed curvature in 1/m.
    double targetSpeed { 0.0 }; ///< Optimal target velocity in m/s.
};

/// @class SpeedProfiler2D
/// @brief Optimal curvature-constrained two-pass dynamic velocity profiler.
/// @details Synthesizes continuous target speed profiles along Spline2D trajectories.
/// Evaluates cornering apex speeds from local curvature and executes backward deceleration
/// and forward traction sweeps with closed-loop cyclic boundary relaxation.
class SpeedProfiler2D {
public:
    /// @brief Default constructor with baseline sports car limits.
    SpeedProfiler2D() noexcept = default;

    /// @brief Parameterized constructor using explicit vehicle physical limits.
    /// @param[in] limits Kinematic and traction boundary limits.
    explicit SpeedProfiler2D(const SpeedProfileLimits& limits) noexcept;

    /// @brief Updates kinematic and traction limits.
    /// @param[in] limits New limit parameters.
    void setLimits(const SpeedProfileLimits& limits) noexcept;

    /// @brief Gets active kinematic and traction limits.
    /// @return Reference to active limits.
    [[nodiscard]] const SpeedProfileLimits& getLimits() const noexcept;

    /// @brief Generates an optimal discrete speed profile along a spline curve.
    /// @param[in] spline Target trajectory spline.
    /// @return True if profile was successfully constructed.
    [[nodiscard]] bool buildProfile(const Spline2D& spline) noexcept;

    /// @brief Queries target velocity for a specified arc-length distance.
    /// @param[in] distance Arc-length distance from spline start in meters.
    /// @return Linearly interpolated target speed in m/s.
    [[nodiscard]] double getSpeedAt(double distance) const noexcept;

    /// @brief Gets total synthesized milestones.
    /// @return Read-only reference to milestones vector.
    [[nodiscard]] const std::vector<SpeedMilestone>& getMilestones() const noexcept;

    /// @brief Clears current milestone array and resets profiler.
    void clear() noexcept;

private:
    SpeedProfileLimits mLimits {};
    std::vector<SpeedMilestone> mMilestones {};
    double mTotalLength { 0.0 };
    bool mIsClosed { false };
};

} // namespace vehicle_sim::math
