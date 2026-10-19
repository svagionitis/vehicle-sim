#include "apps/MotionLab/CircuitTrackCoordinator.h"

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace motion_lab {

namespace {

    constexpr double kGravity { 9.80665 };

    [[nodiscard]] double crossProduct(const vehicle_sim::math::Vec2& a, const vehicle_sim::math::Vec2& b) noexcept
    {
        return a.x * b.y - a.y * b.x;
    }

} // namespace

CircuitTrackCoordinator::CircuitTrackCoordinator() noexcept
    : mActiveMaterial { vehicle_sim::terrain::SurfaceMaterial::getPreset(vehicle_sim::terrain::MaterialType::Tarmac) }
{
    regenerate(mSeed);
}

void CircuitTrackCoordinator::regenerate(std::uint32_t seed) noexcept
{
    mSeed = seed;
    const double trackW { std::max({ 10.0, 3.5 * mVehicleWidth + 1.0, (mVehicleLength >= 10.0) ? 12.0 : 10.0 }) };
    const double shoulderW { std::max(4.0, 1.5 * mVehicleWidth) };
    const double scaleFactor { std::max(1.0, mVehicleLength / 6.0) };

    vehicle_sim::terrain::CircuitTrackConfig config {};
    config.radiusX = 100.0 * scaleFactor;
    config.radiusY = 70.0 * scaleFactor;
    config.waypointCount = 24U;
    config.trackWidth = trackW;
    config.shoulderWidth = shoulderW;
    config.perturbation = 0.25;
    config.enableBanking = true;
    config.designSpeed = 25.0;
    config.maxBankAngle = 0.35;
    config.seed = seed;
    config.enableHazards = mEnableHazards;
    config.hazardConfig = mHazardConfig;
    config.hazardConfig.seed = seed + 500U;

    mTrack = vehicle_sim::terrain::TrackGenerator::generateCircuitTrack(config);
    mMetrics = CircuitLapMetrics {};
    mHasStartedLap = false;
    mCurrentProgress = 0.0;
}

void CircuitTrackCoordinator::stepPhysics(vehicle_sim::physics::RigidBody2D& body, double dt) noexcept
{
    if (!mEnabled) {
        return;
    }

    const vehicle_sim::math::Vec2 currPos { body.getPosition() };

    evaluateSurface(body);
    evaluateBanking(body);

    // Dynamic hazard triggers (e.g. Boost Pad speed impulse)
    for (const auto& h : mTrack.hazards) {
        if (h.type == vehicle_sim::terrain::HazardType::BoostPad) {
            const double distSq { (currPos - h.position).lengthSquared() };
            if (distSq < (h.boundingRadius * h.boundingRadius)) {
                const vehicle_sim::math::Vec2 boostDir { std::cos(h.orientation), std::sin(h.orientation) };
                body.applyLinearImpulse(boostDir * (body.getMass() * h.severity * dt * 2.0));
            }
        }
    }

    if (mHasStartedLap) {
        mMetrics.currentLapTime += dt;
    }

    if (mLastVehiclePos.lengthSquared() > 1e-6) {
        checkTimingGates(mLastVehiclePos, currPos);
    }
    mLastVehiclePos = currPos;
    mCurrentProgress = mTrack.getTrackProgress(currPos);
}

void CircuitTrackCoordinator::resetToGrid(vehicle_sim::physics::RigidBody2D& body) noexcept
{
    body.setPosition(mTrack.startGridPosition);
    body.setOrientation(mTrack.startHeading);
    body.setLinearVelocity(vehicle_sim::math::Vec2::zero());
    body.setAngularVelocity(0.0);
    body.clearForces();

    mLastVehiclePos = mTrack.startGridPosition;
    mMetrics.currentLapTime = 0.0;
    mMetrics.currentSector = 1U;
    mHasStartedLap = false;
    mCurrentProgress = 0.0;
    mActiveMaterial = vehicle_sim::terrain::SurfaceMaterial::getPreset(vehicle_sim::terrain::MaterialType::Tarmac);
}

bool CircuitTrackCoordinator::isEnabled() const noexcept
{
    return mEnabled;
}

void CircuitTrackCoordinator::toggleEnabled() noexcept
{
    mEnabled = !mEnabled;
}

void CircuitTrackCoordinator::setEnabled(bool enabled) noexcept
{
    mEnabled = enabled;
}

const vehicle_sim::terrain::CircuitTrack2D& CircuitTrackCoordinator::getTrack() const noexcept
{
    return mTrack;
}

const CircuitLapMetrics& CircuitTrackCoordinator::getMetrics() const noexcept
{
    return mMetrics;
}

vehicle_sim::terrain::SurfaceMaterial CircuitTrackCoordinator::getActiveMaterial() const noexcept
{
    return mActiveMaterial;
}

double CircuitTrackCoordinator::getTrackProgress() const noexcept
{
    return mCurrentProgress;
}

void CircuitTrackCoordinator::evaluateSurface(vehicle_sim::physics::RigidBody2D& body) noexcept
{
    const double dist { mTrack.getDistanceToCenter(body.getPosition()) };
    const double halfTrack { 0.5 * mTrack.trackWidth };
    const double halfShoulder { halfTrack + mTrack.shoulderWidth };

    if (dist <= halfTrack) {
        mActiveMaterial = vehicle_sim::terrain::SurfaceMaterial::getPreset(vehicle_sim::terrain::MaterialType::Tarmac);
    } else if (dist <= halfShoulder) {
        mActiveMaterial = vehicle_sim::terrain::SurfaceMaterial::getPreset(vehicle_sim::terrain::MaterialType::Gravel);
    } else {
        mActiveMaterial = vehicle_sim::terrain::SurfaceMaterial::getPreset(vehicle_sim::terrain::MaterialType::Grass);
    }
}

void CircuitTrackCoordinator::evaluateBanking(vehicle_sim::physics::RigidBody2D& body) noexcept
{
    const vehicle_sim::math::Vec2 pos { body.getPosition() };
    const double dist { mTrack.getDistanceToCenter(pos) };
    if (dist > (0.5 * mTrack.trackWidth)) {
        return;
    }

    const double bankAngle { mTrack.getBankAngle(pos) };
    if (std::abs(bankAngle) < 1e-4) {
        return;
    }

    const std::size_t numPoints { mTrack.centerline.size() };
    if (numPoints < 2U) {
        return;
    }

    // Find nearest segment tangent
    std::size_t bestIdx { 0U };
    double bestDistSq { std::numeric_limits<double>::infinity() };
    for (std::size_t i { 0U }; i < numPoints; ++i) {
        const double dSq { (mTrack.centerline[i] - pos).lengthSquared() };
        if (dSq < bestDistSq) {
            bestDistSq = dSq;
            bestIdx = i;
        }
    }

    const std::size_t nextIdx { (bestIdx + 1U) % numPoints };
    const vehicle_sim::math::Vec2 forward { (mTrack.centerline[nextIdx] - mTrack.centerline[bestIdx]).normalized() };
    // Transversal normal pointing left (outward if counter-clockwise)
    const vehicle_sim::math::Vec2 normal { -forward.y, forward.x };

    // Gravitational component along bank plane: F = mass * g * sin(bankAngle)
    const double bankForceMag { body.getMass() * kGravity * std::sin(bankAngle) };
    body.applyForce(normal * bankForceMag);
}

void CircuitTrackCoordinator::checkTimingGates(
    const vehicle_sim::math::Vec2& prevPos, const vehicle_sim::math::Vec2& currPos) noexcept
{
    const std::size_t numPoints { mTrack.centerline.size() };
    if (numPoints < 6U || mTrack.leftBoundary.size() != numPoints || mTrack.rightBoundary.size() != numPoints) {
        return;
    }

    const std::size_t startGateIdx { 0U };
    const std::size_t sector1GateIdx { numPoints / 3U };
    const std::size_t sector2GateIdx { (2U * numPoints) / 3U };

    const auto checkGate = [&](std::size_t idx) -> bool {
        const vehicle_sim::math::Vec2& gateL { mTrack.leftBoundary[idx] };
        const vehicle_sim::math::Vec2& gateR { mTrack.rightBoundary[idx] };
        if (!checkSegmentCross(prevPos, currPos, gateL, gateR)) {
            return false;
        }
        const std::size_t nextIdx { (idx + 1U) % numPoints };
        const vehicle_sim::math::Vec2 fwd { mTrack.centerline[nextIdx] - mTrack.centerline[idx] };
        const vehicle_sim::math::Vec2 move { currPos - prevPos };
        return (move.dot(fwd) > 0.0);
    };

    // Check Start / Finish gate
    if (checkGate(startGateIdx)) {
        if (mHasStartedLap) {
            mMetrics.sector3Time
                = std::max(0.0, mMetrics.currentLapTime - (mMetrics.sector1Time + mMetrics.sector2Time));
            mMetrics.lastLapTime = mMetrics.currentLapTime;
            if (!mMetrics.hasValidBest || mMetrics.lastLapTime < mMetrics.bestLapTime) {
                mMetrics.bestLapTime = mMetrics.lastLapTime;
                mMetrics.hasValidBest = true;
            }
            ++mMetrics.completedLaps;
            mMetrics.currentLapTime = 0.0;
            mMetrics.currentSector = 1U;
        } else {
            mHasStartedLap = true;
            mMetrics.currentLapTime = 0.0;
            mMetrics.currentSector = 1U;
        }
    } else if (mHasStartedLap && mMetrics.currentSector == 1U && checkGate(sector1GateIdx)) {
        mMetrics.sector1Time = mMetrics.currentLapTime;
        mMetrics.currentSector = 2U;
    } else if (mHasStartedLap && mMetrics.currentSector == 2U && checkGate(sector2GateIdx)) {
        mMetrics.sector2Time = std::max(0.0, mMetrics.currentLapTime - mMetrics.sector1Time);
        mMetrics.currentSector = 3U;
    }
}

bool CircuitTrackCoordinator::checkSegmentCross(const vehicle_sim::math::Vec2& a1, const vehicle_sim::math::Vec2& a2,
    const vehicle_sim::math::Vec2& b1, const vehicle_sim::math::Vec2& b2) noexcept
{
    const vehicle_sim::math::Vec2 da { a2 - a1 };
    const vehicle_sim::math::Vec2 db { b2 - b1 };

    const double cp1 { crossProduct(da, b1 - a1) };
    const double cp2 { crossProduct(da, b2 - a1) };
    const double cp3 { crossProduct(db, a1 - b1) };
    const double cp4 { crossProduct(db, a2 - b1) };

    const bool straddlesA { (cp1 * cp2) <= 0.0 };
    const bool straddlesB { (cp3 * cp4) <= 0.0 };

    return straddlesA && straddlesB;
}

void CircuitTrackCoordinator::updateDimensions(double vehicleWidth, double vehicleLength) noexcept
{
    mVehicleWidth = std::max(0.5, vehicleWidth);
    mVehicleLength = std::max(1.0, vehicleLength);
    regenerate(mSeed);
}

double CircuitTrackCoordinator::getTrackWidth() const noexcept
{
    return mTrack.trackWidth;
}

double CircuitTrackCoordinator::getVehicleWidth() const noexcept
{
    return mVehicleWidth;
}

double CircuitTrackCoordinator::getVehicleLength() const noexcept
{
    return mVehicleLength;
}

void CircuitTrackCoordinator::setHazardsEnabled(bool enabled) noexcept
{
    mEnableHazards = enabled;
    regenerate(mSeed);
}

bool CircuitTrackCoordinator::isHazardsEnabled() const noexcept
{
    return mEnableHazards;
}

void CircuitTrackCoordinator::setHazardConfig(const vehicle_sim::terrain::HazardScatteringConfig& config) noexcept
{
    mHazardConfig = config;
    if (mEnableHazards) {
        regenerate(mSeed);
    }
}

const vehicle_sim::terrain::HazardScatteringConfig& CircuitTrackCoordinator::getHazardConfig() const noexcept
{
    return mHazardConfig;
}

} // namespace motion_lab
