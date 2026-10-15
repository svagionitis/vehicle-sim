#pragma once

#include <cstddef>
#include <vector>

#include "libs/VehicleSimCore/math/Vec2.h"

namespace motion_lab {

/// @struct SkidSegment
/// @brief 2D line segment representing a tire skid mark.
struct SkidSegment {
    vehicle_sim::math::Vec2 from { 0.0, 0.0 }; ///< Segment start in world coordinates.
    vehicle_sim::math::Vec2 to { 0.0, 0.0 };   ///< Segment end in world coordinates.
    double age { 0.0 };                         ///< Elapsed time since creation in seconds.
};

/// @class SkidRibbonBuffer
/// @brief Circular buffer managing rear tire skidmark trail geometry and aging.
/// @details Records contact path segments when rear tires exceed lateral skid thresholds,
/// and fades segment opacity linearly over the configured skidmark lifetime.
class SkidRibbonBuffer {
public:
    static constexpr std::size_t kMaxSegments { 2600 };

    /// @brief Default constructor with nominal 10-second lifetime.
    /// @param[in] lifetime Skidmark lifetime in seconds.
    explicit SkidRibbonBuffer(double lifetime = 10.0) noexcept;

    /// @brief Clears all active skidmark segments and previous tire states.
    void clear() noexcept;

    /// @brief Updates segment aging and purges segments exceeding lifetime.
    /// @param[in] dt Timestep duration in seconds.
    void update(double dt) noexcept;

    /// @brief Records a new skidmark segment if endpoints are non-degenerate.
    /// @param[in] from Segment starting coordinates in world meters.
    /// @param[in] to Segment ending coordinates in world meters.
    void addSegment(const vehicle_sim::math::Vec2& from,
                    const vehicle_sim::math::Vec2& to) noexcept;

    /// @brief Gets active skidmark segments.
    /// @return Constant reference to vector of SkidSegment.
    [[nodiscard]] const std::vector<SkidSegment>& getSegments() const noexcept;

    /// @brief Gets skidmark lifetime in seconds.
    [[nodiscard]] double getLifetime() const noexcept;

    /// @brief Sets skidmark lifetime in seconds.
    /// @param[in] lifetime New lifetime duration.
    void setLifetime(double lifetime) noexcept;

private:
    std::vector<SkidSegment> mSegments { };
    double mLifetime { 10.0 };
};

} // namespace motion_lab
