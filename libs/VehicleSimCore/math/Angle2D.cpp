#include "Angle2D.h"

#include <algorithm>
#include <cmath>

namespace vehicle_sim::math {

Angle2D Angle2D::fromRadians(double radians) noexcept
{
    return Angle2D { normalizeRad(radians) };
}

Angle2D Angle2D::fromDegrees(double degrees) noexcept
{
    return Angle2D { normalizeRad(degToRad(degrees)) };
}

Angle2D Angle2D::fromVector(const Vec2& vec) noexcept
{
    return Angle2D { toAngle(vec) };
}

Vec2 Angle2D::toVector() const noexcept
{
    return fromAngle(mRadians);
}

Angle2D Angle2D::operator-() const noexcept
{
    return Angle2D { normalizeRad(-mRadians) };
}

Angle2D Angle2D::operator+(const Angle2D& rhs) const noexcept
{
    return Angle2D { normalizeRad(mRadians + rhs.mRadians) };
}

Angle2D Angle2D::operator-(const Angle2D& rhs) const noexcept
{
    return Angle2D { normalizeRad(mRadians - rhs.mRadians) };
}

Angle2D& Angle2D::operator+=(const Angle2D& rhs) noexcept
{
    mRadians = normalizeRad(mRadians + rhs.mRadians);
    return *this;
}

Angle2D& Angle2D::operator-=(const Angle2D& rhs) noexcept
{
    mRadians = normalizeRad(mRadians - rhs.mRadians);
    return *this;
}

double Angle2D::normalizeRad(double angle) noexcept
{
    if (!std::isfinite(angle)) {
        return 0.0;
    }
    const double r { std::atan2(std::sin(angle), std::cos(angle)) };
    return (r == -0.0) ? 0.0 : r;
}

double Angle2D::normalizeDeg(double angle) noexcept
{
    if (!std::isfinite(angle)) {
        return 0.0;
    }
    const double r { radToDeg(normalizeRad(degToRad(angle))) };
    return (r == -0.0) ? 0.0 : r;
}

double Angle2D::normalizeRad0To2Pi(double angle) noexcept
{
    if (!std::isfinite(angle)) {
        return 0.0;
    }
    double r { std::fmod(angle, kTwoPi) };
    if (r < 0.0) {
        r += kTwoPi;
    }
    if (r >= kTwoPi || r == -0.0) {
        r = 0.0;
    }
    return r;
}

double Angle2D::deltaAngle(double from, double to) noexcept
{
    return normalizeRad(to - from);
}

double Angle2D::angularDistance(double a, double b) noexcept
{
    return std::abs(deltaAngle(a, b));
}

double Angle2D::lerpAngle(double a, double b, double t) noexcept
{
    return normalizeRad(a + (t * deltaAngle(a, b)));
}

Vec2 Angle2D::slerp2D(const Vec2& a, const Vec2& b, double t) noexcept
{
    const double lenA { a.length() };
    const double lenB { b.length() };
    if (lenA <= 1e-12 || lenB <= 1e-12) {
        return (a * (1.0 - t)) + (b * t);
    }

    const Vec2 uA { a / lenA };
    const Vec2 uB { b / lenB };
    const double len { (lenA * (1.0 - t)) + (lenB * t) };

    double dotProd { (uA.x * uB.x) + (uA.y * uB.y) };
    if (dotProd > 1.0) {
        dotProd = 1.0;
    } else if (dotProd < -1.0) {
        dotProd = -1.0;
    }

    if (dotProd > 0.999999) {
        const Vec2 blended { (uA * (1.0 - t)) + (uB * t) };
        return blended.normalized() * len;
    }

    if (dotProd < -0.999999) {
        const Vec2 perp { -uA.y, uA.x };
        const double theta { kPi * t };
        const Vec2 dir { (uA * std::cos(theta)) + (perp * std::sin(theta)) };
        return dir * len;
    }

    const double omega { std::acos(dotProd) };
    const double sinOmega { std::sin(omega) };
    const double factorA { std::sin((1.0 - t) * omega) / sinOmega };
    const double factorB { std::sin(t * omega) / sinOmega };
    const Vec2 dir { (uA * factorA) + (uB * factorB) };
    return dir * len;
}

Vec2 Angle2D::fromAngle(double radians) noexcept
{
    return Vec2 { std::cos(radians), std::sin(radians) };
}

double Angle2D::toAngle(const Vec2& vec) noexcept
{
    return std::atan2(vec.y, vec.x);
}

double Angle2D::signedAngleBetween(const Vec2& from, const Vec2& to) noexcept
{
    if (from.lengthSquared() <= 1e-24 || to.lengthSquared() <= 1e-24) {
        return 0.0;
    }
    return std::atan2(from.cross(to), from.dot(to));
}

double Angle2D::clampAngle(double angle, double minAngle, double maxAngle) noexcept
{
    if (!std::isfinite(angle)) {
        return 0.0;
    }
    if (!std::isfinite(minAngle) || !std::isfinite(maxAngle)) {
        return normalizeRad(angle);
    }

    const double span { normalizeRad0To2Pi(maxAngle - minAngle) };
    if (span <= 1e-12) {
        return normalizeRad(minAngle);
    }

    const double offset { normalizeRad0To2Pi(angle - minAngle) };
    if (offset <= span) {
        return normalizeRad(angle);
    }

    const double distToMax { offset - span };
    const double distToMin { kTwoPi - offset };
    if (distToMax <= distToMin) {
        return normalizeRad(maxAngle);
    }
    return normalizeRad(minAngle);
}

} // namespace vehicle_sim::math
