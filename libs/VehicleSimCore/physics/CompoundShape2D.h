#pragma once

#include <array>
#include <cstddef>
#include <variant>

#include "libs/VehicleSimCore/math/AABB2D.h"
#include "libs/VehicleSimCore/math/Capsule2D.h"
#include "libs/VehicleSimCore/math/Circle2D.h"
#include "libs/VehicleSimCore/math/OBB2D.h"
#include "libs/VehicleSimCore/math/Polygon2D.h"
#include "libs/VehicleSimCore/math/Transform2D.h"

namespace vehicle_sim::physics {

/// @struct CompoundChild2D
/// @brief Child primitive shape positioned at a local coordinate transform.
struct CompoundChild2D {
    using Shape = std::variant<math::Circle2D, math::OBB2D, math::Capsule2D, math::Polygon2D>;
    Shape shape { math::OBB2D {} };
    math::Transform2D localTransform {};
};

/// @class CompoundShape2D
/// @brief Multi-primitive compound volume aggregating up to 4 child shapes.
/// @details Designed with fixed capacity to guarantee zero dynamic heap allocations.
/// Enables complex vehicle hulls (stepped truck cabs, multi-box beds, trailers).
class CompoundShape2D {
public:
    static constexpr std::size_t kMaxChildren { 4 };

    CompoundShape2D() noexcept = default;

    /// @brief Adds a child primitive shape with relative local transform.
    /// @param[in] shape Geometric shape variant.
    /// @param[in] localTransform Position and orientation offset relative to parent body.
    /// @return True if child was added, false if capacity exceeded.
    bool addChild(const CompoundChild2D::Shape& shape, const math::Transform2D& localTransform) noexcept;

    /// @brief Clears all registered child shapes.
    void clear() noexcept;

    /// @brief Gets the number of registered child shapes.
    /// @return Child shape count in range [0, kMaxChildren].
    [[nodiscard]] std::size_t getChildCount() const noexcept;

    /// @brief Gets a child shape descriptor by index.
    /// @param[in] index Child index (wrapped modulo childCount).
    /// @return Const reference to child descriptor.
    [[nodiscard]] const CompoundChild2D& getChild(std::size_t index) const noexcept;

    /// @brief Computes enclosing world AABB given the parent body world transform.
    /// @param[in] bodyTransform World transform of the parent rigid body.
    /// @return Enclosing world space AABB.
    [[nodiscard]] math::AABB2D computeWorldAABB(const math::Transform2D& bodyTransform) const noexcept;

private:
    std::array<CompoundChild2D, kMaxChildren> mChildren {};
    std::size_t mCount { 0 };
};

} // namespace vehicle_sim::physics
