#include "libs/VehicleSimCore/dynamics/TrajectoryTracker2D.h"

#include <algorithm>
#include <cmath>

namespace vehicle_sim::dynamics {

TrajectoryTracker2D::TrajectoryTracker2D() noexcept
{
    mSpeedPid.setOutputLimits(-1.0, 1.0);
    mSpeedPid.setIntegralLimits(-0.5, 0.5);
}

void TrajectoryTracker2D::configure(const math::StanleyConfig& stanleyConfig,
    const math::SpeedProfileLimits& profileLimits, const math::PIDGains& speedGains, double maxSteerRate) noexcept
{
    mStanley.setConfig(stanleyConfig);
    mProfiler.setLimits(profileLimits);
    mSpeedPid.setGains(speedGains);
    mSteerLimiter.setRateLimit(std::max(0.1, maxSteerRate));

    if (mPath != nullptr) {
        (void)mProfiler.buildProfile(*mPath);
    }
}

bool TrajectoryTracker2D::setTrajectory(const math::Spline2D* spline) noexcept
{
    mPath = spline;
    mStanley.setPath(spline);
    reset();

    if (spline == nullptr) {
        mProfiler.clear();
        return false;
    }

    return mProfiler.buildProfile(*spline);
}

void TrajectoryTracker2D::reset() noexcept
{
    mSpeedPid.reset();
    mSteerLimiter.reset(0.0);
}

AutopilotOutput TrajectoryTracker2D::update(
    double dt, const math::Vec2& cogPosition, double headingRad, double forwardSpeed, double yawRate) noexcept
{
    AutopilotOutput output {};
    if (mPath == nullptr) {
        return output;
    }

    const math::StanleyState stanleyState { mStanley.computeSteer(cogPosition, headingRad, forwardSpeed, yawRate) };

    output.crossTrackError = stanleyState.crossTrackError;
    output.headingError = stanleyState.headingError;

    const double rawSteer { stanleyState.steerAngle };
    const double limitedSteer { mSteerLimiter.update(rawSteer, dt) };
    output.steerAngleRad = limitedSteer;

    const double maxSteerLock { std::max(0.01, mStanley.getConfig().maxSteerAngle) };
    output.normalizedSteer = std::clamp(limitedSteer / maxSteerLock, -1.0, 1.0);

    const double targetSpeed { mProfiler.getSpeedAt(stanleyState.pathDistance) };
    output.targetSpeed = targetSpeed;

    const double pidEffort { mSpeedPid.update(targetSpeed, forwardSpeed, dt) };
    if (pidEffort >= 0.0) {
        output.throttle = std::clamp(pidEffort, 0.0, 1.0);
        output.brake = 0.0;
    } else {
        output.throttle = 0.0;
        output.brake = std::clamp(-pidEffort, 0.0, 1.0);
    }

    return output;
}

const math::StanleyController& TrajectoryTracker2D::getStanley() const noexcept
{
    return mStanley;
}

const math::SpeedProfiler2D& TrajectoryTracker2D::getProfiler() const noexcept
{
    return mProfiler;
}

const math::PIDController& TrajectoryTracker2D::getSpeedPid() const noexcept
{
    return mSpeedPid;
}

} // namespace vehicle_sim::dynamics
