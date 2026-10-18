#include "apps/MontsoftOffroad/Camera2D.h"

#include <algorithm>
#include <cmath>

namespace montsoft_offroad {

using vehicle_sim::math::Vec2;

Camera2D::Camera2D() noexcept
    : Camera2D(CameraConfig{})
{
}

Camera2D::Camera2D(const CameraConfig& config) noexcept
    : config_{ config }
{
}

void Camera2D::update(const Vec2& targetPos, double forwardSpeed, double dt) noexcept
{
    if (dt <= 0.0) {
        return;
    }

    const double leadDist { std::clamp(
        forwardSpeed * config_.lookaheadLead,
        0.0,
        config_.maxLookahead
    ) };

    const Vec2 desiredPos{ targetPos.x + leadDist, targetPos.y };

    const double alphaX { 1.0 - std::exp(-config_.followRateX * dt) };
    const double alphaY { 1.0 - std::exp(-config_.followRateY * dt) };

    position_.x += alphaX * (desiredPos.x - position_.x);
    position_.y += alphaY * (desiredPos.y - position_.y);

    if (trauma_ > 0.0) {
        trauma_ = std::max(0.0, trauma_ - config_.traumaDecayRate * dt);
        shakeTime_ += dt;

        constexpr double kFrequency { 24.0 };
        constexpr double kPi { 3.14159265358979323846 };
        const double shakeMagnitude { trauma_ * trauma_ * config_.maxShakeDisplacement };

        shakeOffset_.x = shakeMagnitude * std::sin(2.0 * kPi * kFrequency * shakeTime_);
        shakeOffset_.y = shakeMagnitude * std::cos(2.0 * kPi * kFrequency * 1.3 * shakeTime_);
    } else {
        shakeOffset_ = Vec2{ 0.0, 0.0 };
    }
}

void Camera2D::reset(const Vec2& targetPos) noexcept
{
    position_ = targetPos;
    shakeOffset_ = Vec2{ 0.0, 0.0 };
    trauma_ = 0.0;
    shakeTime_ = 0.0;
}

Vec2 Camera2D::worldToScreen(const Vec2& worldPos) const noexcept
{
    const Vec2 effectivePos{ position_.x + shakeOffset_.x, position_.y + shakeOffset_.y };

    const double anchorScreenX { config_.screenFractionX * static_cast<double>(config_.viewportWidth) };
    const double anchorScreenY { config_.screenFractionY * static_cast<double>(config_.viewportHeight) };

    const double screenX { anchorScreenX + (worldPos.x - effectivePos.x) * config_.pixelsPerMeter };
    const double screenY { anchorScreenY - (worldPos.y - effectivePos.y) * config_.pixelsPerMeter };

    return Vec2{ screenX, screenY };
}

Vec2 Camera2D::screenToWorld(const Vec2& screenPos) const noexcept
{
    const Vec2 effectivePos{ position_.x + shakeOffset_.x, position_.y + shakeOffset_.y };

    const double anchorScreenX { config_.screenFractionX * static_cast<double>(config_.viewportWidth) };
    const double anchorScreenY { config_.screenFractionY * static_cast<double>(config_.viewportHeight) };

    const double worldX { effectivePos.x + (screenPos.x - anchorScreenX) / config_.pixelsPerMeter };
    const double worldY { effectivePos.y + (anchorScreenY - screenPos.y) / config_.pixelsPerMeter };

    return Vec2{ worldX, worldY };
}

void Camera2D::addTrauma(double trauma) noexcept
{
    trauma_ = std::clamp(trauma_ + std::max(0.0, trauma), 0.0, 1.0);
}

double Camera2D::getVisibleMinX() const noexcept
{
    return screenToWorld(Vec2{ 0.0, 0.0 }).x;
}

double Camera2D::getVisibleMaxX() const noexcept
{
    return screenToWorld(Vec2{ static_cast<double>(config_.viewportWidth), 0.0 }).x;
}

double Camera2D::getVisibleMinY() const noexcept
{
    return screenToWorld(Vec2{ 0.0, static_cast<double>(config_.viewportHeight) }).y;
}

double Camera2D::getVisibleMaxY() const noexcept
{
    return screenToWorld(Vec2{ 0.0, 0.0 }).y;
}

const Vec2& Camera2D::getPosition() const noexcept
{
    return position_;
}

const Vec2& Camera2D::getShakeOffset() const noexcept
{
    return shakeOffset_;
}

double Camera2D::getTrauma() const noexcept
{
    return trauma_;
}

const CameraConfig& Camera2D::getConfig() const noexcept
{
    return config_;
}

void Camera2D::setConfig(const CameraConfig& config) noexcept
{
    config_ = config;
}

} // namespace montsoft_offroad
