#pragma once

#include <cmath>
#include <optional>

#include "Vec2.h"

namespace vehicle_sim::math {

/// @class Mat2
/// @brief 2x2 matrix for 2D linear transformations and rotations.
/// @details Represents a 2x2 row-major matrix:
/// [ m00  m01 ]
/// [ m10  m11 ]
class Mat2 {
public:
    double m00 { 1.0 };
    double m01 { 0.0 };
    double m10 { 0.0 };
    double m11 { 1.0 };

    /// @brief Default constructor initializing to identity matrix.
    constexpr Mat2() noexcept = default;

    /// @brief Parameterized constructor initializing individual elements.
    /// @param[in] in00 Row 0, Col 0 element.
    /// @param[in] in01 Row 0, Col 1 element.
    /// @param[in] in10 Row 1, Col 0 element.
    /// @param[in] in11 Row 1, Col 1 element.
    constexpr Mat2(double in00, double in01, double in10, double in11) noexcept
        : m00 { in00 }
        , m01 { in01 }
        , m10 { in10 }
        , m11 { in11 }
    {
    }

    /// @brief Returns the 2x2 identity matrix.
    /// @return Identity matrix.
    [[nodiscard]] static constexpr Mat2 identity() noexcept
    {
        return Mat2 { 1.0, 0.0, 0.0, 1.0 };
    }

    /// @brief Returns a 2x2 zero matrix.
    /// @return Zero matrix.
    [[nodiscard]] static constexpr Mat2 zero() noexcept
    {
        return Mat2 { 0.0, 0.0, 0.0, 0.0 };
    }

    /// @brief Creates a rotation matrix for a given angle in radians.
    /// @param[in] angleRadians Rotation angle in radians.
    /// @return Rotation matrix.
    [[nodiscard]] static Mat2 fromAngle(double angleRadians) noexcept;

    /// @brief Creates a matrix from two row vectors.
    /// @param[in] row0 First row vector.
    /// @param[in] row1 Second row vector.
    /// @return Constructed matrix.
    [[nodiscard]] static constexpr Mat2 fromRows(const Vec2& row0, const Vec2& row1) noexcept
    {
        return Mat2 { row0.x, row0.y, row1.x, row1.y };
    }

    /// @brief Creates a matrix from two column vectors.
    /// @param[in] col0 First column vector.
    /// @param[in] col1 Second column vector.
    /// @return Constructed matrix.
    [[nodiscard]] static constexpr Mat2 fromColumns(const Vec2& col0, const Vec2& col1) noexcept
    {
        return Mat2 { col0.x, col1.x, col0.y, col1.y };
    }

    /// @brief Matrix addition operator.
    /// @param[in] rhs Right-hand matrix.
    /// @return Sum of matrices.
    [[nodiscard]] constexpr Mat2 operator+(const Mat2& rhs) const noexcept
    {
        return Mat2 { m00 + rhs.m00, m01 + rhs.m01, m10 + rhs.m10, m11 + rhs.m11 };
    }

    /// @brief Matrix subtraction operator.
    /// @param[in] rhs Right-hand matrix.
    /// @return Difference of matrices.
    [[nodiscard]] constexpr Mat2 operator-(const Mat2& rhs) const noexcept
    {
        return Mat2 { m00 - rhs.m00, m01 - rhs.m01, m10 - rhs.m10, m11 - rhs.m11 };
    }

    /// @brief Scalar multiplication operator.
    /// @param[in] scalar Multiplier scalar.
    /// @return Scaled matrix.
    [[nodiscard]] constexpr Mat2 operator*(double scalar) const noexcept
    {
        return Mat2 { m00 * scalar, m01 * scalar, m10 * scalar, m11 * scalar };
    }

    /// @brief Matrix multiplication operator.
    /// @param[in] rhs Right-hand matrix.
    /// @return Matrix product.
    [[nodiscard]] constexpr Mat2 operator*(const Mat2& rhs) const noexcept
    {
        return Mat2 { (m00 * rhs.m00) + (m01 * rhs.m10), (m00 * rhs.m01) + (m01 * rhs.m11),
            (m10 * rhs.m00) + (m11 * rhs.m10), (m10 * rhs.m01) + (m11 * rhs.m11) };
    }

    /// @brief Vector transformation operator (M * v).
    /// @param[in] v Vector to transform.
    /// @return Transformed vector.
    [[nodiscard]] constexpr Vec2 operator*(const Vec2& v) const noexcept
    {
        return Vec2 { (m00 * v.x) + (m01 * v.y), (m10 * v.x) + (m11 * v.y) };
    }

    /// @brief Exact equality comparison operator.
    /// @param[in] rhs Other matrix.
    /// @return True if all elements are identical.
    [[nodiscard]] constexpr bool operator==(const Mat2& rhs) const noexcept
    {
        return (m00 == rhs.m00) && (m01 == rhs.m01) && (m10 == rhs.m10) && (m11 == rhs.m11);
    }

    /// @brief Exact inequality comparison operator.
    /// @param[in] rhs Other matrix.
    /// @return True if any element differs.
    [[nodiscard]] constexpr bool operator!=(const Mat2& rhs) const noexcept
    {
        return !(*this == rhs);
    }

    /// @brief Computes the determinant of the matrix.
    /// @return Determinant scalar.
    [[nodiscard]] constexpr double det() const noexcept
    {
        return (m00 * m11) - (m01 * m10);
    }

    /// @brief Computes the transpose of the matrix.
    /// @return Transposed matrix.
    [[nodiscard]] constexpr Mat2 transposed() const noexcept
    {
        return Mat2 { m00, m10, m01, m11 };
    }

    /// @brief Computes the inverse of the matrix.
    /// @param[in] epsilon Numerical singularity threshold.
    /// @return Inverse matrix, or std::nullopt if matrix is singular.
    [[nodiscard]] std::optional<Mat2> inverse(double epsilon = 1e-12) const noexcept;

    /// @brief Checks if matrix is approximately equal to another matrix.
    /// @param[in] other Matrix to compare.
    /// @param[in] epsilon Numerical tolerance.
    /// @return True if each element differs by at most epsilon.
    [[nodiscard]] bool isApproxEqual(const Mat2& other, double epsilon = 1e-9) const noexcept;
};

/// @brief Scalar left-multiplication operator.
/// @param[in] scalar Multiplier scalar.
/// @param[in] mat Matrix to scale.
/// @return Scaled matrix.
[[nodiscard]] constexpr Mat2 operator*(double scalar, const Mat2& mat) noexcept
{
    return mat * scalar;
}

} // namespace vehicle_sim::math
