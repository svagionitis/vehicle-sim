#include "libs/VehicleSimCore/math/StanleyController.h"

#include <algorithm>
#include <cmath>

namespace vehicle_sim::math {

StanleyController::StanleyController(const StanleyConfig& config) noexcept
    : mConfig { config }
{
}

void StanleyController::setPath(const Spline2D* path) noexcept
{
    mPath = path;
}

void StanleyController::setConfig(const StanleyConfig& config) noexcept
{
    mConfig = config;
}

const StanleyConfig& StanleyController::getConfig() const noexcept
{
    return mConfig;
}

const Spline2D* StanleyController::getPath() const noexcept
{
    return mPath;
}

Vec2 StanleyController::getFrontAxle(const Vec2& cogPosition, double headingRad) const noexcept
{
    const Vec2 headingVec { std::cos(headingRad), std::sin(headingRad) };
    return cogPosition + (headingVec * mConfig.frontAxleOffset);
}

StanleyState StanleyController::computeSteer(
    const Vec2& cogPosition, double headingRad, double forwardSpeed, double yawRate) const noexcept
{
    StanleyState state {};
    const Vec2 frontAxle { getFrontAxle(cogPosition, headingRad) };
    state.frontAxlePos = frontAxle;

    if (mPath == nullptr || mPath->getPointCount() < 2) {
        return state;
    }

    const SplineClosestPoint closest { mPath->findClosest(frontAxle) };
    state.closestPoint = closest.point;
    state.pathDistance = closest.arcLength;

    const SplineSample2D sample { mPath->evaluateSample(closest.parameter) };
    const double pathHeading { std::atan2(sample.tangent.y, sample.tangent.x) };

    const double signedOffset { closest.signedOffset };
    state.crossTrackError = signedOffset;

    const bool isReversing { forwardSpeed < -0.2 };
    const double effectiveSpeed { std::max(std::abs(forwardSpeed), std::max(mConfig.minVelocity, 0.05)) };

    double headingError { 0.0 };
    double steerCmd { 0.0 };

    if (!isReversing) {
        headingError = Angle2D::deltaAngle(headingRad, pathHeading);
        state.headingError = headingError;

        const double crossTrackTerm { -std::atan2(mConfig.crossTrackGain * signedOffset, effectiveSpeed) };
        const double dampingTerm { -mConfig.dampingGain * yawRate };

        steerCmd = (mConfig.headingGain * headingError) + crossTrackTerm + dampingTerm;
    } else {
        const double reverseHeading { Angle2D::normalizeRad(headingRad + Angle2D::kPi) };
        headingError = Angle2D::deltaAngle(reverseHeading, pathHeading);
        state.headingError = headingError;

        const double crossTrackTerm { std::atan2(mConfig.crossTrackGain * signedOffset, effectiveSpeed) };
        const double dampingTerm { mConfig.dampingGain * yawRate };

        steerCmd = (mConfig.headingGain * headingError) + crossTrackTerm + dampingTerm;
    }

    const double maxSteer { std::max(0.01, mConfig.maxSteerAngle) };
    state.steerAngle = std::clamp(steerCmd, -maxSteer, maxSteer);

    return state;
}

} // namespace vehicle_sim::math
