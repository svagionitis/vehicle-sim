#include "apps/MotionLab/MotionLabCamera.h"

#include <algorithm>
#include <cmath>

namespace motion_lab {

MotionLabCamera::MotionLabCamera() noexcept
    : MotionLabCamera(CameraConfig { })
{
}

MotionLabCamera::MotionLabCamera(const CameraConfig& config) noexcept
    : mConfig { config }
    , mZoom { config.baseZoom }
    , mBaseZoom { config.baseZoom }
{
    resize(config.viewportWidth, config.viewportHeight);
}

void MotionLabCamera::reset(const vehicle_sim::math::Vec2& position) noexcept
{
    mCamera = position;
    mZoom = mBaseZoom;
    mVelocityZoomFactor = 1.0;
    mSimulationTime = 0.0;
    mSpeedHistory.clear();
}

vehicle_sim::math::Vec2 MotionLabCamera::worldToScreen(const vehicle_sim::math::Vec2& worldPos) const noexcept
{
    const double screenX { (static_cast<double>(mConfig.viewportWidth) * 0.5) + (worldPos.x - mCamera.x) * mZoom };
    const double screenY { (static_cast<double>(mConfig.viewportHeight) * 0.5) - (worldPos.y - mCamera.y) * mZoom };
    return vehicle_sim::math::Vec2 { screenX, screenY };
}

vehicle_sim::math::Vec2 MotionLabCamera::screenToWorld(const vehicle_sim::math::Vec2& screenPos) const noexcept
{
    const double worldX { mCamera.x + (screenPos.x - static_cast<double>(mConfig.viewportWidth) * 0.5) / mZoom };
    const double worldY { mCamera.y - (screenPos.y - static_cast<double>(mConfig.viewportHeight) * 0.5) / mZoom };
    return vehicle_sim::math::Vec2 { worldX, worldY };
}

vehicle_sim::physics::BarrierBounds2D MotionLabCamera::getBounds() const noexcept
{
    const double inset { mConfig.barrierInsetPx / mZoom };
    const double halfWidth { (static_cast<double>(mConfig.viewportWidth) / (2.0 * mZoom)) - inset };
    const double halfHeight { (static_cast<double>(mConfig.viewportHeight) / (2.0 * mZoom)) - inset };

    return vehicle_sim::physics::BarrierBounds2D {
        mCamera.x - halfWidth,
        mCamera.x + halfWidth,
        mCamera.y - halfHeight,
        mCamera.y + halfHeight
    };
}

void MotionLabCamera::resize(int width, int height) noexcept
{
    mConfig.viewportWidth = std::max(100, width);
    mConfig.viewportHeight = std::max(100, height);

    const double shortSide { static_cast<double>(std::min(mConfig.viewportWidth, mConfig.viewportHeight)) };
    const double viewportMaxZoom { shortSide < 600.0 ? 32.0 : 28.0 };

    // Fit nominal 10m span inside viewport leaving 32px margin
    const double fitZoom { (shortSide - 32.0) / 10.0 };
    mBaseZoom = std::max(1.0, std::min(viewportMaxZoom, fitZoom));

    // Keep at least 5 grid cells (5m each) visible across short axis
    const double gridVisibilityZoom { shortSide / 25.0 };
    if (shortSide > 0.0) {
        mBaseZoom = std::min(mBaseZoom, gridVisibilityZoom);
    }

    mZoom = mBaseZoom * mVelocityZoomFactor;
}

double MotionLabCamera::computeAverageSpeed(double currentSpeed, double dt) noexcept
{
    mSimulationTime += dt;
    mSpeedHistory.push_back({ mSimulationTime, currentSpeed });

    const double cutoff { mSimulationTime - mConfig.speedHistoryWindow };
    while (mSpeedHistory.size() > 2 && mSpeedHistory[1].time <= cutoff) {
        mSpeedHistory.erase(mSpeedHistory.begin());
    }

    if (mSpeedHistory.size() < 2) {
        return currentSpeed;
    }

    double weightedSpeed { 0.0 };
    double elapsed { 0.0 };
    for (std::size_t i { 0 }; i + 1 < mSpeedHistory.size(); ++i) {
        const double start { std::max(cutoff, mSpeedHistory[i].time) };
        const double end { std::min(mSimulationTime, mSpeedHistory[i + 1].time) };
        if (end <= start) {
            continue;
        }
        const double duration { end - start };
        weightedSpeed += mSpeedHistory[i].speed * duration;
        elapsed += duration;
    }

    return (elapsed > 0.0) ? (weightedSpeed / elapsed) : currentSpeed;
}

void MotionLabCamera::updateCamera(const vehicle_sim::math::Vec2& position, double dt) noexcept
{
    if (mFixedCamera) {
        return;
    }

    if (!mDeadZoneEnabled) {
        mCamera = position;
        return;
    }

    const double halfDeadWidth { (static_cast<double>(mConfig.viewportWidth) * mConfig.cameraDeadZone) / mZoom };
    const double halfDeadHeight { (static_cast<double>(mConfig.viewportHeight) * mConfig.cameraDeadZone) / mZoom };

    const double offsetX { std::clamp(position.x - mCamera.x, -halfDeadWidth, halfDeadWidth) };
    const double offsetY { std::clamp(position.y - mCamera.y, -halfDeadHeight, halfDeadHeight) };

    const vehicle_sim::math::Vec2 target { position.x - offsetX, position.y - offsetY };
    const double follow { 1.0 - std::exp(-mConfig.cameraFollowRate * dt) };

    mCamera += (target - mCamera) * follow;
}

void MotionLabCamera::updateZoom(const vehicle_sim::math::Vec2& position,
                                const vehicle_sim::math::Vec2& velocity,
                                double dt) noexcept
{
    if (mFixedCamera) {
        return;
    }

    const double speed { computeAverageSpeed(velocity.length(), dt) };
    double speedFactor { 0.0 };

    if (mSpeedZoomEnabled) {
        const double range { mConfig.maxVelocityZoomSpeed - mConfig.minVelocityZoomSpeed };
        if (range > 1e-4) {
            speedFactor = std::clamp((speed - mConfig.minVelocityZoomSpeed) / range, 0.0, 1.0);
        }
    }

    const double targetFactor { 1.0 - mConfig.maxVelocityZoomOut * speedFactor };
    const double blend { 1.0 - std::exp(-mConfig.zoomRate * dt) };
    mVelocityZoomFactor += (targetFactor - mVelocityZoomFactor) * blend;

    const double previousZoom { mZoom };
    mZoom = mBaseZoom * mVelocityZoomFactor;

    // Zoom around body anchor to prevent shifts from pulling vehicle across viewport
    if (mZoom > 1e-4) {
        const double anchor { previousZoom / mZoom };
        mCamera.x = position.x - (position.x - mCamera.x) * anchor;
        mCamera.y = position.y - (position.y - mCamera.y) * anchor;
    }
}

void MotionLabCamera::update(const vehicle_sim::math::Vec2& position,
                            const vehicle_sim::math::Vec2& velocity,
                            double dt) noexcept
{
    if (dt <= 0.0) {
        return;
    }
    updateCamera(position, dt);
    updateZoom(position, velocity, dt);
}

const vehicle_sim::math::Vec2& MotionLabCamera::getPosition() const noexcept
{
    return mCamera;
}

double MotionLabCamera::getZoom() const noexcept
{
    return mZoom;
}

double MotionLabCamera::getBaseZoom() const noexcept
{
    return mBaseZoom;
}

void MotionLabCamera::setPosition(const vehicle_sim::math::Vec2& pos) noexcept
{
    mCamera = pos;
}

bool MotionLabCamera::isDeadZoneEnabled() const noexcept
{
    return mDeadZoneEnabled;
}

void MotionLabCamera::setDeadZoneEnabled(bool enabled) noexcept
{
    mDeadZoneEnabled = enabled;
}

bool MotionLabCamera::isSpeedZoomEnabled() const noexcept
{
    return mSpeedZoomEnabled;
}

void MotionLabCamera::setSpeedZoomEnabled(bool enabled) noexcept
{
    mSpeedZoomEnabled = enabled;
}

bool MotionLabCamera::isFixedCamera() const noexcept
{
    return mFixedCamera;
}

void MotionLabCamera::setFixedCamera(bool fixed) noexcept
{
    mFixedCamera = fixed;
}

const CameraConfig& MotionLabCamera::getConfig() const noexcept
{
    return mConfig;
}

} // namespace motion_lab
