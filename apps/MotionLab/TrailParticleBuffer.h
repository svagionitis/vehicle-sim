#pragma once

#include <cstddef>
#include <vector>

#include "libs/VehicleSimCore/math/Vec2.h"

namespace motion_lab {

/// @struct TrailDot
/// @brief 2D world coordinate dot representing vehicle center breadcrumb trail.
struct TrailDot {
    vehicle_sim::math::Vec2 position { 0.0, 0.0 }; ///< Dot world coordinates.
    double age { 0.0 };                             ///< Dot age in seconds.
};

/// @class TrailParticleBuffer
/// @brief Manages chassis breadcrumb trail emission and quadratic opacity decay.
/// @details Emits position samples at fixed 30 Hz intervals if vehicle moves,
/// and maintains a circular buffer of up to 800 historical points.
class TrailParticleBuffer {
public:
    static constexpr std::size_t kMaxDots { 800 };

    /// @brief Default constructor with nominal 12-second lifetime.
    /// @param[in] lifetime Trail dot lifetime in seconds.
    explicit TrailParticleBuffer(double lifetime = 12.0) noexcept;

    /// @brief Clears all active trail dots and resets emission timer.
    void clear() noexcept;

    /// @brief Updates dot aging, emits new dots if moved, and purges expired dots.
    /// @param[in] position Vehicle center of mass coordinates in world meters.
    /// @param[in] dt Simulation timestep duration in seconds.
    void update(const vehicle_sim::math::Vec2& position, double dt) noexcept;

    /// @brief Gets active trail dots.
    /// @return Constant reference to vector of TrailDot.
    [[nodiscard]] const std::vector<TrailDot>& getDots() const noexcept;

    /// @brief Gets trail lifetime in seconds.
    [[nodiscard]] double getLifetime() const noexcept;

    /// @brief Sets trail lifetime in seconds.
    /// @param[in] lifetime New trail lifetime (0 disables trail).
    void setLifetime(double lifetime) noexcept;

private:
    std::vector<TrailDot> mDots { };
    double mLifetime { 12.0 };
    double mTrailClock { 0.0 };
    vehicle_sim::math::Vec2 mLastPosition { 0.0, 0.0 };
    bool mHasLastPosition { false };
};

} // namespace motion_lab
