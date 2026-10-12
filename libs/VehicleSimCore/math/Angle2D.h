#pragma once

#include <cmath>

#include "libs/VehicleSimCore/math/Vec2.h"

namespace vehicle_sim::math {

/// @class Angle2D
/// @brief 2D angular representation and trigonometric mathematical primitives.
/// @details Represents a planar angle in radians normalized to the IEEE 754 principal domain [-pi, pi).
/// Provides branchless O(1) angular normalization, shortest-path angular difference, circular interpolation,
/// spherical vector interpolation (slerp), and directional conversions with zero external dependencies.
class Angle2D {
public:
    /// @brief Mathematical constant Pi (3.14159265358979323846).
    static constexpr double kPi { 3.14159265358979323846 };

    /// @brief Mathematical constant Two Pi (6.28318530717958647692).
    static constexpr double kTwoPi { 6.28318530717958647692 };

    /// @brief Mathematical constant Half Pi (1.57079632679489661923).
    static constexpr double kHalfPi { 1.57079632679489661923 };

    /// @brief Degrees-to-radians conversion factor (Pi / 180.0).
    static constexpr double kDegToRad { kPi / 180.0 };

    /// @brief Radians-to-degrees conversion factor (180.0 / Pi).
    static constexpr double kRadToDeg { 180.0 / kPi };

    /// @brief Default constructor initializing angle to zero radians.
    constexpr Angle2D() noexcept = default;

    /// @brief Factory creating an Angle2D from radians, normalized into [-pi, pi).
    /// @param[in] radians Input angle in radians.
    /// @return Normalized Angle2D instance.
    [[nodiscard]] static Angle2D fromRadians(double radians) noexcept;

    /// @brief Factory creating an Angle2D from degrees, normalized into [-pi, pi).
    /// @param[in] degrees Input angle in degrees.
    /// @return Normalized Angle2D instance.
    [[nodiscard]] static Angle2D fromDegrees(double degrees) noexcept;

    /// @brief Factory creating an Angle2D from a 2D Cartesian vector direction.
    /// @param[in] vec Input direction vector (x, y).
    /// @return Normalized Angle2D instance representing atan2(y, x).
    [[nodiscard]] static Angle2D fromVector(const Vec2& vec) noexcept;

    /// @brief Returns an Angle2D representing zero radians.
    /// @return Zero angle instance.
    [[nodiscard]] static constexpr Angle2D zero() noexcept
    {
        return Angle2D { 0.0 };
    }

    /// @brief Returns an Angle2D representing -Pi radians.
    /// @return Inverted boundary angle instance.
    [[nodiscard]] static constexpr Angle2D pi() noexcept
    {
        return Angle2D { -kPi };
    }

    /// @brief Gets the stored angle in radians.
    /// @return Angle in radians within [-pi, pi).
    [[nodiscard]] constexpr double radians() const noexcept
    {
        return mRadians;
    }

    /// @brief Gets the stored angle converted to degrees.
    /// @return Angle in degrees within [-180, 180).
    [[nodiscard]] double degrees() const noexcept
    {
        return mRadians * kRadToDeg;
    }

    /// @brief Converts this angle to a unit Cartesian direction vector.
    /// @return Unit vector (cos(angle), sin(angle)).
    [[nodiscard]] Vec2 toVector() const noexcept;

    /// @brief Unary negation operator.
    /// @return Negated normalized Angle2D.
    [[nodiscard]] Angle2D operator-() const noexcept;

    /// @brief Angular addition operator.
    /// @param[in] rhs Other angle to add.
    /// @return Sum of angles normalized to [-pi, pi).
    [[nodiscard]] Angle2D operator+(const Angle2D& rhs) const noexcept;

    /// @brief Angular subtraction operator.
    /// @param[in] rhs Other angle to subtract.
    /// @return Difference of angles normalized to [-pi, pi).
    [[nodiscard]] Angle2D operator-(const Angle2D& rhs) const noexcept;

    /// @brief In-place angular addition.
    /// @param[in] rhs Other angle to add.
    /// @return Reference to this angle.
    Angle2D& operator+=(const Angle2D& rhs) noexcept;

    /// @brief In-place angular subtraction.
    /// @param[in] rhs Other angle to subtract.
    /// @return Reference to this angle.
    Angle2D& operator-=(const Angle2D& rhs) noexcept;

    /// @brief Exact equality comparison operator.
    /// @param[in] rhs Other angle to compare.
    /// @return True if normalized radian values are identical.
    [[nodiscard]] constexpr bool operator==(const Angle2D& rhs) const noexcept
    {
        return mRadians == rhs.mRadians;
    }

    /// @brief Exact inequality comparison operator.
    /// @param[in] rhs Other angle to compare.
    /// @return True if normalized radian values differ.
    [[nodiscard]] constexpr bool operator!=(const Angle2D& rhs) const noexcept
    {
        return mRadians != rhs.mRadians;
    }

    /// @brief Normalizes an angle in radians into the principal domain [-pi, pi).
    /// @details Uses IEEE 754 remainder arithmetic for deterministic O(1) performance without looping.
    /// @param[in] angle Input angle in radians.
    /// @return Equivalent angle wrapped into [-pi, pi).
    [[nodiscard]] static double normalizeRad(double angle) noexcept;

    /// @brief Normalizes an angle in degrees into [-180, 180).
    /// @param[in] angle Input angle in degrees.
    /// @return Equivalent angle wrapped into [-180, 180).
    [[nodiscard]] static double normalizeDeg(double angle) noexcept;

    /// @brief Normalizes an angle in radians into the positive domain [0, 2*pi).
    /// @param[in] angle Input angle in radians.
    /// @return Equivalent angle wrapped into [0, 2*pi).
    [[nodiscard]] static double normalizeRad0To2Pi(double angle) noexcept;

    /// @brief Computes the minimal signed angular difference from one angle to another.
    /// @details Returns the signed delta in [-pi, pi) such that normalizeRad(from + delta) == to.
    /// Positive values represent counter-clockwise rotation; negative values represent clockwise rotation.
    /// @param[in] from Starting angle in radians.
    /// @param[in] to Destination angle in radians.
    /// @return Shortest signed difference in radians within [-pi, pi).
    [[nodiscard]] static double deltaAngle(double from, double to) noexcept;

    /// @brief Computes the unsigned absolute angular distance between two angles.
    /// @param[in] a First angle in radians.
    /// @param[in] b Second angle in radians.
    /// @return Unsigned shortest distance in radians within [0, pi].
    [[nodiscard]] static double angularDistance(double a, double b) noexcept;

    /// @brief Linearly interpolates between angles along the shortest circular arc.
    /// @details Avoids full-circle revolution artifacts across the -pi/pi branch cut.
    /// @param[in] a Starting angle in radians.
    /// @param[in] b Target angle in radians.
    /// @param[in] t Interpolation parameter in [0, 1].
    /// @return Interpolated angle in radians within [-pi, pi).
    [[nodiscard]] static double lerpAngle(double a, double b, double t) noexcept;

    /// @brief Spherical linear interpolation between two 2D vectors along the unit arc.
    /// @details Interpolates both direction and magnitude smoothly. Handles parallel and antipodal cases.
    /// @param[in] a Starting vector.
    /// @param[in] b Target vector.
    /// @param[in] t Interpolation parameter in [0, 1].
    /// @return Interpolated vector.
    [[nodiscard]] static Vec2 slerp2D(const Vec2& a, const Vec2& b, double t) noexcept;

    /// @brief Constructs a unit Cartesian vector from an angle in radians.
    /// @param[in] radians Input angle in radians.
    /// @return Unit vector (cos(radians), sin(radians)).
    [[nodiscard]] static Vec2 fromAngle(double radians) noexcept;

    /// @brief Computes the polar angle of a 2D Cartesian vector.
    /// @param[in] vec Input vector.
    /// @return Angle in radians within [-pi, pi) using atan2.
    [[nodiscard]] static double toAngle(const Vec2& vec) noexcept;

    /// @brief Computes the minimal signed angle from one vector to another.
    /// @param[in] from Origin vector.
    /// @param[in] to Destination vector.
    /// @return Signed angle in radians within [-pi, pi).
    [[nodiscard]] static double signedAngleBetween(const Vec2& from, const Vec2& to) noexcept;

    /// @brief Clamps an angle within a circular boundary range [minAngle, maxAngle].
    /// @param[in] angle Input angle in radians.
    /// @param[in] minAngle Lower boundary angle in radians.
    /// @param[in] maxAngle Upper boundary angle in radians.
    /// @return Clamped angle in radians within [-pi, pi).
    [[nodiscard]] static double clampAngle(double angle, double minAngle, double maxAngle) noexcept;

    /// @brief Converts degrees to radians.
    /// @param[in] deg Angle in degrees.
    /// @return Angle in radians.
    [[nodiscard]] static constexpr double degToRad(double deg) noexcept
    {
        return deg * kDegToRad;
    }

    /// @brief Converts radians to degrees.
    /// @param[in] rad Angle in radians.
    /// @return Angle in degrees.
    [[nodiscard]] static constexpr double radToDeg(double rad) noexcept
    {
        return rad * kRadToDeg;
    }

private:
    explicit constexpr Angle2D(double normalizedRad) noexcept
        : mRadians { normalizedRad }
    {
    }

    double mRadians { 0.0 };
};

} // namespace vehicle_sim::math
