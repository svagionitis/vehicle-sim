#pragma once

#include <cmath>
#include <limits>

namespace vehicle_sim::math {

/// @class Vec2
/// @brief 2D Cartesian vector supporting linear algebra operations.
/// @details Represents a 2D Euclidean vector (x, y) with IEEE 754 double-precision arithmetic.
class Vec2 {
public:
    double x { 0.0 };
    double y { 0.0 };

    /// @brief Default constructor initializing to (0, 0).
    constexpr Vec2() noexcept = default;

    /// @brief Parameterized constructor.
    /// @param[in] inX X coordinate component.
    /// @param[in] inY Y coordinate component.
    constexpr Vec2(double inX, double inY) noexcept
        : x { inX }
        , y { inY }
    {
    }

    /// @brief Returns a zero vector (0, 0).
    /// @return Zero vector.
    [[nodiscard]] static constexpr Vec2 zero() noexcept
    {
        return Vec2 { 0.0, 0.0 };
    }

    /// @brief Returns a unit vector along the X axis (1, 0).
    /// @return Unit X vector.
    [[nodiscard]] static constexpr Vec2 unitX() noexcept
    {
        return Vec2 { 1.0, 0.0 };
    }

    /// @brief Returns a unit vector along the Y axis (0, 1).
    /// @return Unit Y vector.
    [[nodiscard]] static constexpr Vec2 unitY() noexcept
    {
        return Vec2 { 0.0, 1.0 };
    }

    /// @brief Unary negation operator.
    /// @return Negated vector.
    [[nodiscard]] constexpr Vec2 operator-() const noexcept
    {
        return Vec2 { -x, -y };
    }

    /// @brief Vector addition operator.
    /// @param[in] rhs Right-hand vector to add.
    /// @return Sum of vectors.
    [[nodiscard]] constexpr Vec2 operator+(const Vec2& rhs) const noexcept
    {
        return Vec2 { x + rhs.x, y + rhs.y };
    }

    /// @brief Vector subtraction operator.
    /// @param[in] rhs Right-hand vector to subtract.
    /// @return Difference of vectors.
    [[nodiscard]] constexpr Vec2 operator-(const Vec2& rhs) const noexcept
    {
        return Vec2 { x - rhs.x, y - rhs.y };
    }

    /// @brief Scalar multiplication operator.
    /// @param[in] scalar Multiplier scalar.
    /// @return Scaled vector.
    [[nodiscard]] constexpr Vec2 operator*(double scalar) const noexcept
    {
        return Vec2 { x * scalar, y * scalar };
    }

    /// @brief Scalar division operator.
    /// @param[in] scalar Divisor scalar.
    /// @return Scaled vector.
    [[nodiscard]] constexpr Vec2 operator/(double scalar) const noexcept
    {
        const double inv { 1.0 / scalar };
        return Vec2 { x * inv, y * inv };
    }

    /// @brief In-place vector addition.
    /// @param[in] rhs Right-hand vector to add.
    /// @return Reference to this vector.
    constexpr Vec2& operator+=(const Vec2& rhs) noexcept
    {
        x += rhs.x;
        y += rhs.y;
        return *this;
    }

    /// @brief In-place vector subtraction.
    /// @param[in] rhs Right-hand vector to subtract.
    /// @return Reference to this vector.
    constexpr Vec2& operator-=(const Vec2& rhs) noexcept
    {
        x -= rhs.x;
        y -= rhs.y;
        return *this;
    }

    /// @brief In-place scalar multiplication.
    /// @param[in] scalar Multiplier scalar.
    /// @return Reference to this vector.
    constexpr Vec2& operator*=(double scalar) noexcept
    {
        x *= scalar;
        y *= scalar;
        return *this;
    }

    /// @brief In-place scalar division.
    /// @param[in] scalar Divisor scalar.
    /// @return Reference to this vector.
    constexpr Vec2& operator/=(double scalar) noexcept
    {
        const double inv { 1.0 / scalar };
        x *= inv;
        y *= inv;
        return *this;
    }

    /// @brief Exact equality comparison operator.
    /// @param[in] rhs Other vector to compare.
    /// @return True if coordinates are identical.
    [[nodiscard]] constexpr bool operator==(const Vec2& rhs) const noexcept
    {
        return (x == rhs.x) && (y == rhs.y);
    }

    /// @brief Exact inequality comparison operator.
    /// @param[in] rhs Other vector to compare.
    /// @return True if coordinates differ.
    [[nodiscard]] constexpr bool operator!=(const Vec2& rhs) const noexcept
    {
        return !(*this == rhs);
    }

    /// @brief Computes the dot product with another vector.
    /// @param[in] other Other vector.
    /// @return Dot product scalar.
    [[nodiscard]] constexpr double dot(const Vec2& other) const noexcept
    {
        return (x * other.x) + (y * other.y);
    }

    /// @brief Computes the 2D cross product (perp-dot product) with another vector.
    /// @details Returns the scalar z-component of the 3D cross product: (x * other.y) - (y * other.x).
    /// @param[in] other Other vector.
    /// @return Cross product scalar.
    [[nodiscard]] constexpr double cross(const Vec2& other) const noexcept
    {
        return (x * other.y) - (y * other.x);
    }

    /// @brief Computes squared Euclidean length.
    /// @return Length squared.
    [[nodiscard]] constexpr double lengthSquared() const noexcept
    {
        return (x * x) + (y * y);
    }

    /// @brief Computes Euclidean length.
    /// @return Length magnitude.
    [[nodiscard]] double length() const noexcept
    {
        return std::hypot(x, y);
    }

    /// @brief Returns a normalized unit vector.
    /// @details If length is below threshold, returns zero vector.
    /// @param[in] eps Minimum length threshold.
    /// @return Unit vector or zero vector.
    [[nodiscard]] Vec2 normalized(double eps = 1e-12) const noexcept;

    /// @brief Normalizes this vector in-place.
    /// @param[in] eps Minimum length threshold.
    void normalize(double eps = 1e-12) noexcept;

    /// @brief Returns a counter-clockwise perpendicular vector (-y, x).
    /// @return Perpendicular vector.
    [[nodiscard]] constexpr Vec2 perpendicular() const noexcept
    {
        return Vec2 { -y, x };
    }

    /// @brief Rotates vector by given angle in radians.
    /// @param[in] angleRadians Angle of rotation in radians.
    /// @return Rotated vector.
    [[nodiscard]] Vec2 rotated(double angleRadians) const noexcept;

    /// @brief Checks if vector is approximately equal to another vector.
    /// @param[in] other Vector to compare.
    /// @param[in] epsilon Numerical tolerance.
    /// @return True if differences in x and y are within epsilon.
    [[nodiscard]] bool isApproxEqual(const Vec2& other, double epsilon = 1e-9) const noexcept;

    /// @brief Checks if vector length is near zero.
    /// @param[in] epsilon Numerical tolerance.
    /// @return True if length squared is less than epsilon squared.
    [[nodiscard]] bool isZero(double epsilon = 1e-9) const noexcept;
};

/// @brief Scalar left-multiplication operator.
/// @param[in] scalar Multiplier scalar.
/// @param[in] vec Vector to multiply.
/// @return Scaled vector.
[[nodiscard]] constexpr Vec2 operator*(double scalar, const Vec2& vec) noexcept
{
    return vec * scalar;
}

/// @brief Computes the dot product of two vectors.
/// @param[in] lhs First vector.
/// @param[in] rhs Second vector.
/// @return Dot product scalar.
[[nodiscard]] constexpr double dot(const Vec2& lhs, const Vec2& rhs) noexcept
{
    return lhs.dot(rhs);
}

/// @brief Computes the 2D cross product of two vectors.
/// @param[in] lhs First vector.
/// @param[in] rhs Second vector.
/// @return Cross product scalar.
[[nodiscard]] constexpr double cross(const Vec2& lhs, const Vec2& rhs) noexcept
{
    return lhs.cross(rhs);
}

/// @brief Computes cross product of angular velocity scalar and 2D vector (omega x r).
/// @details Used in rigid body dynamics: v = omega x r = (-omega * r.y, omega * r.x).
/// @param[in] omega Angular velocity scalar.
/// @param[in] r Position vector.
/// @return Tangential linear velocity vector.
[[nodiscard]] constexpr Vec2 cross(double omega, const Vec2& r) noexcept
{
    return Vec2 { -omega * r.y, omega * r.x };
}

/// @brief Computes cross product of 2D vector and angular velocity scalar (r x omega).
/// @param[in] r Position vector.
/// @param[in] omega Angular velocity scalar.
/// @return Tangential vector.
[[nodiscard]] constexpr Vec2 cross(const Vec2& r, double omega) noexcept
{
    return Vec2 { omega * r.y, -omega * r.x };
}

} // namespace vehicle_sim::math
