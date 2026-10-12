#include "BankedTrack.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace vehicle_sim::terrain {

BankedTrackProfile::BankedTrackProfile(const BankedTrackConfig& config) noexcept
    : mConfig { config }
{
}

void BankedTrackProfile::setConfig(const BankedTrackConfig& config) noexcept
{
    mConfig = config;
}

const BankedTrackConfig& BankedTrackProfile::getConfig() const noexcept
{
    return mConfig;
}

double BankedTrackProfile::computeBankAngle(double curvature) const noexcept
{
    if (!mConfig.enabled || std::abs(curvature) < 1e-9) {
        return 0.0;
    }

    constexpr double kGravity { 9.81 };
    const double speed { std::max(0.0, mConfig.designSpeed) };
    const double absCurvature { std::abs(curvature) };

    // Theoretical equilibrium superelevation: tan(theta) = v^2 / (g * R) = v^2 * kappa / g
    const double tanTheta { (speed * speed * absCurvature) / kGravity };
    const double thetaMag { std::atan(tanTheta) };
    const double maxAngle { std::max(0.0, mConfig.maxBankAngle) };
    const double clampedAngle { std::min(thetaMag, maxAngle) };

    return (curvature < 0.0) ? -clampedAngle : clampedAngle;
}

CamberProfileSample BankedTrackProfile::evaluateCamber(
    double bankAngle, double lateralOffset, double halfWidth, const math::Vec2& tangent) const noexcept
{
    CamberProfileSample sample { };
    const double w { lateralOffset };
    const double safeHalfWidth { std::max(1e-3, halfWidth) };

    // Unit lateral normal pointing left relative to forward tangent: (-Ty, Tx)
    const math::Vec2 latNorm { -tangent.y, tangent.x };
    sample.lateralNormal = latNorm;

    if (!mConfig.enabled || mConfig.camberType == TrackCamberType::Flat) {
        sample.elevationOffset = 0.0;
        sample.bankAngle = 0.0;
        sample.lateralSlope = 0.0;
        sample.surfaceNormal = SurfaceNormal3D { 0.0, 0.0, 1.0 };
        return sample;
    }

    double elevOffset { 0.0 };
    double dhdw { 0.0 };
    double effectiveBank { bankAngle };

    switch (mConfig.camberType) {
    case TrackCamberType::Flat:
        elevOffset = 0.0;
        dhdw = 0.0;
        effectiveBank = 0.0;
        break;

    case TrackCamberType::Superelevation:
        // Linear cross-slope: inner apex (w > 0 for left turn) is lower, outer edge is higher
        elevOffset = -w * std::sin(bankAngle);
        dhdw = -std::sin(bankAngle);
        effectiveBank = bankAngle;
        break;

    case TrackCamberType::Crowned: {
        // Crown cross-fall: symmetric drop on straights, blending to linear superelevation in turns
        const double crownSlope { std::max(0.0, mConfig.crownSlope) };
        const double transAngle { std::max(0.01, std::atan(crownSlope) * 2.0) };
        const double blend { std::clamp(std::abs(bankAngle) / transAngle, 0.0, 1.0) };

        const double crownOffset { -crownSlope * std::abs(w) };
        const double bankOffset { -w * std::sin(bankAngle) };
        elevOffset = (1.0 - blend) * crownOffset + blend * bankOffset;

        const double crownDhdw { -crownSlope * (w >= 0.0 ? 1.0 : -1.0) };
        const double bankDhdw { -std::sin(bankAngle) };
        dhdw = (1.0 - blend) * crownDhdw + blend * bankDhdw;

        effectiveBank = blend * bankAngle;
        break;
    }

    case TrackCamberType::ParabolicBowl: {
        // Quadratic progressive banking: bowl steepens toward the outer perimeter
        const double bowlCoeff { mConfig.bowlCurvature };
        const double normW { w / safeHalfWidth };
        elevOffset = -w * std::sin(bankAngle) + bowlCoeff * (normW * normW) * safeHalfWidth;
        dhdw = -std::sin(bankAngle) + (2.0 * bowlCoeff * normW);
        effectiveBank = bankAngle;
        break;
    }
    }

    sample.elevationOffset = elevOffset;
    sample.bankAngle = effectiveBank;
    sample.lateralSlope = dhdw;

    // 3D unit surface normal from transversal gradient: grad = dhdw * latNorm
    // N = (-grad.x, -grad.y, 1.0) / sqrt(1 + |grad|^2)
    const double gradX { dhdw * latNorm.x };
    const double gradY { dhdw * latNorm.y };
    const double gradLenSq { gradX * gradX + gradY * gradY };
    const double denom { std::sqrt(1.0 + gradLenSq) };

    sample.surfaceNormal.x = -gradX / denom;
    sample.surfaceNormal.y = -gradY / denom;
    sample.surfaceNormal.z = 1.0 / denom;

    return sample;
}

double BankedTrackProfile::computeNeutralSpeed(double bankAngle, double radius, double gravity) noexcept
{
    const double r { std::abs(radius) };
    const double theta { std::abs(bankAngle) };
    const double g { std::max(0.1, gravity) };

    if (r < 1e-6 || theta < 1e-6) {
        return 0.0;
    }

    // Neutral equilibrium speed: v = sqrt(g * R * tan(theta))
    return std::sqrt(g * r * std::tan(theta));
}

double BankedTrackProfile::computeMaxSpeed(double bankAngle, double radius, double friction, double gravity) noexcept
{
    const double r { std::abs(radius) };
    const double theta { std::abs(bankAngle) };
    const double mu { std::max(0.0, friction) };
    const double g { std::max(0.1, gravity) };

    if (r < 1e-6) {
        return 0.0;
    }

    const double tanTheta { std::tan(theta) };
    const double denominator { 1.0 - mu * tanTheta };

    if (denominator <= 1e-9) {
        // Banking angle exceeds friction limit (critical entrapment): infinite holding capacity
        return std::numeric_limits<double>::infinity();
    }

    // Civil engineering max cornering speed: v = sqrt(g * R * (tan(theta) + mu) / (1 - mu * tan(theta)))
    const double numerator { tanTheta + mu };
    return std::sqrt(g * r * (numerator / denominator));
}

} // namespace vehicle_sim::terrain
