#pragma once

#include <cmath>

#include "Mat2.h"
#include "Vec2.h"

namespace vehicle_sim::math {

/// @class Transform2D
/// @brief 2D rigid transform combining translation and rotation.
/// @details Represents coordinate frame transformation between local and world coordinates.
class Transform2D {
public:
    Vec2 position { 0.0, 0.0 };
    Mat2 rotation { Mat2::identity() };

    /// @brief Default constructor initializing to identity transform.
    constexpr Transform2D() noexcept = default;

    /// @brief Constructor with translation and rotation matrix.
    /// @param[in] pos Translation vector.
    /// @param[in] rot Rotation matrix.
    constexpr Transform2D(const Vec2& pos, const Mat2& rot) noexcept
        : position { pos }
        , rotation { rot }
    {
    }

    /// @brief Constructor with translation and orientation angle.
    /// @param[in] pos Translation vector.
    /// @param[in] angleRadians Orientation angle in radians.
    Transform2D(const Vec2& pos, double angleRadians) noexcept;

    /// @brief Returns the identity transform.
    /// @return Identity transform.
    [[nodiscard]] static constexpr Transform2D identity() noexcept
    {
        return Transform2D { Vec2::zero(), Mat2::identity() };
    }

    /// @brief Transforms a local point into world coordinates.
    /// @details Computes: world = position + (rotation * localPoint).
    /// @param[in] localPoint Point in local frame.
    /// @return Point in world frame.
    [[nodiscard]] constexpr Vec2 transformPoint(const Vec2& localPoint) const noexcept
    {
        return position + (rotation * localPoint);
    }

    /// @brief Inverse transforms a world point into local coordinates.
    /// @details Computes: local = rotation^T * (worldPoint - position).
    /// @param[in] worldPoint Point in world frame.
    /// @return Point in local frame.
    [[nodiscard]] constexpr Vec2 inverseTransformPoint(const Vec2& worldPoint) const noexcept
    {
        return rotation.transposed() * (worldPoint - position);
    }

    /// @brief Transforms a local direction or velocity vector into world frame.
    /// @details Computes: world = rotation * localVec (ignores translation).
    /// @param[in] localVec Vector in local frame.
    /// @return Vector in world frame.
    [[nodiscard]] constexpr Vec2 transformVector(const Vec2& localVec) const noexcept
    {
        return rotation * localVec;
    }

    /// @brief Inverse transforms a world vector into local frame.
    /// @details Computes: local = rotation^T * worldVec (ignores translation).
    /// @param[in] worldVec Vector in world frame.
    /// @return Vector in local frame.
    [[nodiscard]] constexpr Vec2 inverseTransformVector(const Vec2& worldVec) const noexcept
    {
        return rotation.transposed() * worldVec;
    }

    /// @brief Computes the inverse rigid transformation.
    /// @return Inverse transform.
    [[nodiscard]] constexpr Transform2D inverse() const noexcept
    {
        const Mat2 invRot { rotation.transposed() };
        const Vec2 invPos { -(invRot * position) };
        return Transform2D { invPos, invRot };
    }

    /// @brief Combines this transform with another child transform.
    /// @param[in] rhs Child transform.
    /// @return Combined transform.
    [[nodiscard]] constexpr Transform2D combine(const Transform2D& rhs) const noexcept
    {
        return Transform2D { position + (rotation * rhs.position), rotation * rhs.rotation };
    }

    /// @brief Transform combination operator.
    /// @param[in] rhs Child transform.
    /// @return Combined transform.
    [[nodiscard]] constexpr Transform2D operator*(const Transform2D& rhs) const noexcept
    {
        return combine(rhs);
    }

    /// @brief Extracts orientation angle in radians.
    /// @return Orientation angle in radians within [-pi, pi].
    [[nodiscard]] double getAngle() const noexcept;

    /// @brief Sets orientation angle in radians.
    /// @param[in] angleRadians Orientation angle in radians.
    void setAngle(double angleRadians) noexcept;

    /// @brief Exact equality comparison operator.
    /// @param[in] rhs Other transform.
    /// @return True if position and rotation are identical.
    [[nodiscard]] constexpr bool operator==(const Transform2D& rhs) const noexcept
    {
        return (position == rhs.position) && (rotation == rhs.rotation);
    }

    /// @brief Exact inequality comparison operator.
    /// @param[in] rhs Other transform.
    /// @return True if position or rotation differs.
    [[nodiscard]] constexpr bool operator!=(const Transform2D& rhs) const noexcept
    {
        return !(*this == rhs);
    }

    /// @brief Checks if transform is approximately equal to another transform.
    /// @param[in] other Transform to compare.
    /// @param[in] epsilon Numerical tolerance.
    /// @return True if differences in position and rotation are within tolerance.
    [[nodiscard]] bool isApproxEqual(const Transform2D& other, double epsilon = 1e-9) const noexcept;
};

} // namespace vehicle_sim::math
