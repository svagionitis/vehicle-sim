#include "OffroadTruckDynamics.h"

#include <algorithm>
#include <cmath>

#include "libs/VehicleSimCore/math/Angle2D.h"
#include "libs/VehicleSimCore/math/MathUtils.h"
#include "libs/VehicleSimCore/physics/Integrator2D.h"
#include "libs/VehicleSimCore/physics/RigidBody2D.h"
#include "libs/VehicleSimCore/terrain/DeformableTerrain.h"
#include "libs/VehicleSimCore/terrain/Heightfield1D.h"

namespace vehicle_sim::dynamics {

namespace {
    inline constexpr double kMinSpeedThreshold { 0.05 };
} // namespace

OffroadTruckDynamics::OffroadTruckDynamics() noexcept
    : OffroadTruckDynamics(OffroadTruckConfig { })
{
}

OffroadTruckDynamics::OffroadTruckDynamics(const OffroadTruckConfig& config) noexcept
    : config_ { config }
{
    reset();
}

void OffroadTruckDynamics::step(physics::RigidBody2D& body, double dt) noexcept
{
    if (dt <= 0.0) {
        return;
    }

    if (jumpCooldown_ > 0.0) {
        jumpCooldown_ = std::max(0.0, jumpCooldown_ - dt);
    }

    if (input_.jump) {
        if (!jumpLatch_ && jumpCooldown_ <= 0.0 && isGrounded()) {
            triggerJump();
            jumpLatch_ = true;
        }
    } else {
        jumpLatch_ = false;
    }

    resolveSuspension(body, dt);

    if (jumpTriggered_) {
        if (isGrounded() && jumpCooldown_ <= 0.0) {
            const double jumpV { (jumpSpeedOverride_ > 0.0) ? jumpSpeedOverride_ : config_.jumpSpeed };
            const double currentVy { body.getLinearVelocity().y };
            const double deltaV { std::max(0.0, jumpV - currentVy) };
            if (deltaV > 0.0) {
                const double impulse { (config_.jumpImpulse > 0.0) ? config_.jumpImpulse : (config_.mass * deltaV) };
                body.applyLinearImpulse(math::Vec2 { 0.0, impulse });
            }

            for (auto& axle : axleStates_) {
                axle.isGrounded = false;
                axle.normalForce = 0.0;
            }
            isAirborne_ = true;
            jumpCooldown_ = 0.35;
        }
        jumpTriggered_ = false;
        jumpSpeedOverride_ = 0.0;
    }

    applyTraction(body);
    applyAeroDrag(body);
    applyAttitudeStabilizer(body);
}

void OffroadTruckDynamics::reset() noexcept
{
    forwardSpeed_ = 0.0;
    isAirborne_ = true;
    jumpTriggered_ = false;
    jumpSpeedOverride_ = 0.0;
    jumpCooldown_ = 0.0;
    jumpLatch_ = false;
    input_ = OffroadTruckInput { };

    for (auto& axle : axleStates_) {
        axle.worldPos = math::Vec2::zero();
        axle.contactPoint = math::Vec2::zero();
        axle.surfaceNormal = math::Vec2 { 0.0, 1.0 };
        axle.compression = 0.0;
        axle.normalForce = 0.0;
        axle.tractionForce = 0.0;
        axle.soilSinkage = 0.0;
        axle.soilDragForce = 0.0;
        axle.isGrounded = false;
    }
}

bool OffroadTruckDynamics::isAirborne() const noexcept
{
    return isAirborne_;
}

bool OffroadTruckDynamics::isGrounded() const noexcept
{
    return !isAirborne_;
}

void OffroadTruckDynamics::setInput(const OffroadTruckInput& input) noexcept
{
    input_ = input;
}

const OffroadTruckInput& OffroadTruckDynamics::getInput() const noexcept
{
    return input_;
}

void OffroadTruckDynamics::setTerrain(const terrain::Heightfield1D* terrain) noexcept
{
    terrain_ = terrain;
}

const terrain::Heightfield1D* OffroadTruckDynamics::getTerrain() const noexcept
{
    return terrain_;
}

void OffroadTruckDynamics::setDeformable(terrain::DeformableTerrain* deformable) noexcept
{
    deformableTerrain_ = deformable;
}

terrain::DeformableTerrain* OffroadTruckDynamics::getDeformable() const noexcept
{
    return deformableTerrain_;
}

void OffroadTruckDynamics::triggerJump(double speed) noexcept
{
    if (jumpCooldown_ <= 0.0) {
        jumpTriggered_ = true;
        jumpSpeedOverride_ = speed;
    }
}

const OffroadAxleState& OffroadTruckDynamics::getAxleState(std::size_t index) const noexcept
{
    return axleStates_.at(index);
}

const OffroadTruckConfig& OffroadTruckDynamics::getConfig() const noexcept
{
    return config_;
}

void OffroadTruckDynamics::setConfig(const OffroadTruckConfig& config) noexcept
{
    config_ = config;
}

double OffroadTruckDynamics::getSpeed() const noexcept
{
    return forwardSpeed_;
}

void OffroadTruckDynamics::resolveSuspension(physics::RigidBody2D& body, double dt) noexcept
{
    static_cast<void>(dt);

    const double theta { body.getOrientation() };
    const double cosTheta { std::cos(theta) };
    const double sinTheta { std::sin(theta) };

    const math::Vec2 forwardDir { cosTheta, sinTheta };
    const math::Vec2 upwardDir { -sinTheta, cosTheta };
    const math::Vec2 downwardDir { sinTheta, -cosTheta };

    // Apply downward gravity force on chassis
    body.applyForce(math::Vec2 { 0.0, -config_.mass * config_.gravity });

    std::size_t groundedCount { 0 };

    for (std::size_t i { 0 }; i < kAxleCount; ++i) {
        auto& axle { axleStates_[i] };
        const auto& axleConfig { config_.axles[i] };

        const math::Vec2 attachLocal { axleConfig.longitudinalOffset, 0.0 };
        const math::Vec2 attachWorld { body.toWorldPoint(attachLocal) };
        const math::Vec2 attachVel { body.getVelocityAtPoint(attachWorld) };

        const math::Vec2 uncompressedHub { attachWorld + (downwardDir * axleConfig.restLength) };
        const double xQuery { uncompressedHub.x };

        double groundY { 0.0 };
        math::Vec2 groundNormal { 0.0, 1.0 };
        terrain::SurfaceMaterial material {};
        if (terrain_ != nullptr) {
            const auto sample { terrain_->sampleSurface(xQuery) };
            groundY = sample.elevation;
            groundNormal = sample.normal;
            material = terrain_->getMaterial(xQuery);
        }

        const double contactElevation { groundY + axleConfig.tireRadius };
        const double penetration { contactElevation - uncompressedHub.y };

        if (penetration > 0.0) {
            const double maxAllowedCompression { axleConfig.restLength * 0.95 };
            const double compression { std::min(penetration, maxAllowedCompression) };
            const math::Vec2 hubPos { uncompressedHub + (upwardDir * compression) };

            const double velAlongStrut { attachVel.dot(upwardDir) };
            const double springForce { axleConfig.springStiffness * compression };
            const double dampingForce { -axleConfig.dampingCoeff * velAlongStrut };

            // Non-linear bump stop engagement
            double bumpStopForce { 0.0 };
            if (compression > axleConfig.bumpStopDistance) {
                const double excessDeflection { compression - axleConfig.bumpStopDistance };
                bumpStopForce = axleConfig.bumpStopStiffness * excessDeflection * excessDeflection;
            }

            const double normalMagnitude { std::max(0.0, springForce + dampingForce + bumpStopForce) };
            const math::Vec2 normalForceVec { groundNormal * normalMagnitude };
            body.applyForceAtPoint(normalForceVec, hubPos);

            axle.worldPos = hubPos;
            axle.contactPoint = math::Vec2 { xQuery, groundY };
            axle.surfaceNormal = groundNormal;
            axle.compression = compression;
            axle.normalForce = normalMagnitude;
            axle.isGrounded = true;
            ++groundedCount;

            // Terramechanic soil compaction sinkage and rut drag evaluation
            if (config_.enableBekkerRutting && normalMagnitude > 1.0 && (material.soil.isDeformable || material.soil.frictionModulus > 1e-4)) {
                const double contactArea { axleConfig.tireWidth * (2.0 * std::sqrt(std::max(0.0, axleConfig.tireRadius * axleConfig.tireRadius - (axleConfig.tireRadius - 0.05) * (axleConfig.tireRadius - 0.05)))) };
                const double effArea { std::max(0.02, contactArea) };
                const double contactPressure { normalMagnitude / effArea };

                const double soilSinkage { terrain::BekkerWongSolver::computeSinkage(contactPressure, axleConfig.tireWidth, material.soil) };
                const double compactionDrag { terrain::BekkerWongSolver::computeCompactionDrag(soilSinkage, axleConfig.tireWidth, material.soil) };

                axle.soilSinkage = soilSinkage;
                axle.soilDragForce = compactionDrag;

                // Oppose chassis velocity with compaction drag force
                if (std::abs(forwardSpeed_) > kMinSpeedThreshold) {
                    const double dragSign { (forwardSpeed_ > 0.0) ? -1.0 : 1.0 };
                    body.applyForce(forwardDir * (dragSign * compactionDrag));
                }
            } else {
                axle.soilSinkage = 0.0;
                axle.soilDragForce = 0.0;
            }
        } else {
            axle.worldPos = uncompressedHub;
            axle.contactPoint = math::Vec2 { xQuery, groundY };
            axle.surfaceNormal = groundNormal;
            axle.compression = 0.0;
            axle.normalForce = 0.0;
            axle.tractionForce = 0.0;
            axle.soilSinkage = 0.0;
            axle.soilDragForce = 0.0;
            axle.isGrounded = false;
        }
    }

    isAirborne_ = (groundedCount == 0);

    // Anti-roll / pitch coupling between front and rear axles
    if (axleStates_[kRearAxle].isGrounded && axleStates_[kFrontAxle].isGrounded) {
        const double diffCompression { axleStates_[kFrontAxle].compression - axleStates_[kRearAxle].compression };
        const double antiPitchTorque { diffCompression * config_.antiRollTorqueGain };
        body.applyTorque(antiPitchTorque);
    }
}

void OffroadTruckDynamics::applyTraction(physics::RigidBody2D& body) noexcept
{
    const double theta { body.getOrientation() };
    const math::Vec2 forwardDir { std::cos(theta), std::sin(theta) };

    const math::Vec2 bodyVel { body.getLinearVelocity() };
    forwardSpeed_ = bodyVel.dot(forwardDir);

    if (isAirborne_) {
        return;
    }

    double totalDemand { 0.0 };
    if (input_.throttle > 0.0) {
        totalDemand += input_.throttle * config_.maxDriveForce;
    } else if (input_.throttle < 0.0) {
        totalDemand += input_.throttle * config_.maxReverseForce;
    }

    if (input_.brake > 0.0) {
        double brakeDirection { 0.0 };
        if (forwardSpeed_ > kMinSpeedThreshold) {
            brakeDirection = -1.0;
        } else if (forwardSpeed_ < -kMinSpeedThreshold) {
            brakeDirection = 1.0;
        } else {
            brakeDirection = -forwardSpeed_ / kMinSpeedThreshold;
        }
        totalDemand += brakeDirection * input_.brake * config_.maxBrakeForce;
    }

    double totalNormalForce { 0.0 };
    for (const auto& axle : axleStates_) {
        if (axle.isGrounded) {
            totalNormalForce += axle.normalForce;
        }
    }

    if (totalNormalForce <= 1e-4) {
        return;
    }

    // 4WD torque distribution: 50% split per axle
    for (std::size_t i { 0 }; i < kAxleCount; ++i) {
        auto& axle { axleStates_[i] };
        if (!axle.isGrounded) {
            axle.tractionForce = 0.0;
            continue;
        }

        double frictionCoeff { config_.groundFriction };
        if (terrain_ != nullptr) {
            frictionCoeff = terrain_->getMaterial(axle.contactPoint.x).friction;
        }

        const double maxTraction { axle.normalForce * frictionCoeff };
        // 50:50 torque split
        const double axleTractionDemand { 0.5 * totalDemand };
        const double clampedTraction { std::clamp(axleTractionDemand, -maxTraction, maxTraction) };

        axle.tractionForce = clampedTraction;

        const math::Vec2 tangentDir { axle.surfaceNormal.y, -axle.surfaceNormal.x };
        const math::Vec2 mountPointWorld { body.toWorldPoint(math::Vec2 { config_.axles[i].longitudinalOffset, 0.0 }) };
        body.applyForceAtPoint(tangentDir * clampedTraction, mountPointWorld);
    }
}

void OffroadTruckDynamics::applyAeroDrag(physics::RigidBody2D& body) noexcept
{
    const math::Vec2 vel { body.getLinearVelocity() };
    const double speedSq { vel.lengthSquared() };

    if (speedSq < 1e-4) {
        return;
    }

    const double speed { std::sqrt(speedSq) };
    const double dragMagnitude { 0.5 * config_.airDensity * config_.dragCoefficient * config_.frontalArea * speedSq };
    const math::Vec2 dragForce { vel * (-dragMagnitude / speed) };

    body.applyForce(dragForce);
}

void OffroadTruckDynamics::applyAttitudeStabilizer(physics::RigidBody2D& body) noexcept
{
    const double currentAngle { body.getOrientation() };
    const double angularVel { body.getAngularVelocity() };
    const double wrappedAngle { math::Angle2D::normalizeRad(currentAngle) };

    const bool needsStabilization { isAirborne_ || (!axleStates_[kRearAxle].isGrounded) ||
                                    (!axleStates_[kFrontAxle].isGrounded) || (std::abs(wrappedAngle) > 0.08) };
    if (!needsStabilization) {
        return;
    }

    const double restoringTorque { -wrappedAngle * config_.pitchStabGain };
    const double dampingTorque { -angularVel * (config_.pitchStabGain * 0.25) };

    body.applyTorque(restoringTorque + dampingTorque);
}

} // namespace vehicle_sim::dynamics
