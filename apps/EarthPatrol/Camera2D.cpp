#include "Camera2D.h"

#include <algorithm>
#include <cmath>

namespace earth_patrol {

Camera2D::Camera2D() noexcept
    : Camera2D(CameraConfig { })
{
}

Camera2D::Camera2D(const CameraConfig& config) noexcept
    : config_ { config }
{
}

void Camera2D::update(const vehicle_sim::math::Vec2& targetPos, double forwardSpeed, double dt) noexcept
{
    if (dt <= 0.0) {
        return;
    }

    const double lead { std::clamp(forwardSpeed / 15.0, 0.0, 1.5) * config_.lookaheadLead };
    const double desiredX { targetPos.x + lead };
    const double desiredY { targetPos.y };

    const double alphaX { 1.0 - std::exp(-config_.horizontalFollowRate * dt) };
    const double alphaY { 1.0 - std::exp(-config_.verticalFollowRate * dt) };

    position_.x += (desiredX - position_.x) * alphaX;
    position_.y += (desiredY - position_.y) * alphaY;

    if (shakeTimer_ > 0.0) {
        shakeTimer_ -= dt;
        if (shakeTimer_ <= 0.0) {
            shakeTimer_ = 0.0;
            shakeOffset_ = vehicle_sim::math::Vec2 { 0.0, 0.0 };
        } else {
            const double trauma { std::clamp(shakeTimer_ / shakeDuration_, 0.0, 1.0) };
            const double curIntensity { shakeIntensity_ * trauma * trauma };
            shakeAngle_ += 35.0 * dt;
            shakeOffset_ = vehicle_sim::math::Vec2 {
                std::cos(shakeAngle_ * 1.3) * curIntensity,
                std::sin(shakeAngle_ * 1.7) * curIntensity
            };
        }
    }
}

void Camera2D::addScreenShake(double intensity, double duration) noexcept
{
    shakeIntensity_ = std::max(shakeIntensity_, intensity);
    shakeDuration_ = std::max(shakeDuration_, duration);
    shakeTimer_ = shakeDuration_;
}

const vehicle_sim::math::Vec2& Camera2D::getShakeOffset() const noexcept
{
    return shakeOffset_;
}

void Camera2D::reset(const vehicle_sim::math::Vec2& targetPos) noexcept
{
    position_ = targetPos;
    shakeTimer_ = 0.0;
    shakeOffset_ = vehicle_sim::math::Vec2 { 0.0, 0.0 };
}

vehicle_sim::math::Vec2 Camera2D::worldToScreen(const vehicle_sim::math::Vec2& worldPos) const noexcept
{
    const double anchorScreenX { static_cast<double>(config_.viewportWidth) * config_.screenFractionX };
    const double anchorScreenY { static_cast<double>(config_.viewportHeight) * config_.screenFractionY };

    const double camX { position_.x + shakeOffset_.x };
    const double camY { position_.y + shakeOffset_.y };

    const double screenX { anchorScreenX + ((worldPos.x - camX) * config_.pixelsPerMeter) };
    const double screenY { anchorScreenY - ((worldPos.y - camY) * config_.pixelsPerMeter) };

    return vehicle_sim::math::Vec2 { screenX, screenY };
}

vehicle_sim::math::Vec2 Camera2D::screenToWorld(const vehicle_sim::math::Vec2& screenPos) const noexcept
{
    const double anchorScreenX { static_cast<double>(config_.viewportWidth) * config_.screenFractionX };
    const double anchorScreenY { static_cast<double>(config_.viewportHeight) * config_.screenFractionY };

    const double camX { position_.x + shakeOffset_.x };
    const double camY { position_.y + shakeOffset_.y };

    const double worldX { camX + ((screenPos.x - anchorScreenX) / config_.pixelsPerMeter) };
    const double worldY { camY + ((anchorScreenY - screenPos.y) / config_.pixelsPerMeter) };

    return vehicle_sim::math::Vec2 { worldX, worldY };
}

double Camera2D::getVisibleMinX() const noexcept
{
    const double anchorScreenX { static_cast<double>(config_.viewportWidth) * config_.screenFractionX };
    return position_.x - (anchorScreenX / config_.pixelsPerMeter);
}

double Camera2D::getVisibleMaxX() const noexcept
{
    const double anchorScreenX { static_cast<double>(config_.viewportWidth) * config_.screenFractionX };
    const double remainingWidth { static_cast<double>(config_.viewportWidth) - anchorScreenX };
    return position_.x + (remainingWidth / config_.pixelsPerMeter);
}

const vehicle_sim::math::Vec2& Camera2D::getPosition() const noexcept
{
    return position_;
}

void Camera2D::setPosition(const vehicle_sim::math::Vec2& pos) noexcept
{
    position_ = pos;
}

const CameraConfig& Camera2D::getConfig() const noexcept
{
    return config_;
}

void Camera2D::setConfig(const CameraConfig& config) noexcept
{
    config_ = config;
}

void Camera2D::setViewportSize(int width, int height) noexcept
{
    config_.viewportWidth = width;
    config_.viewportHeight = height;
}

} // namespace earth_patrol
