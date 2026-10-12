#include "EarthRoverDynamics.h"

#include <algorithm>
#include <cmath>

#include "libs/VehicleSimCore/math/Angle2D.h"
#include "libs/VehicleSimCore/math/MathUtils.h"
#include "libs/VehicleSimCore/physics/Integrator2D.h"
#include "libs/VehicleSimCore/physics/RigidBody2D.h"
#include "libs/VehicleSimCore/terrain/Heightfield1D.h"

namespace vehicle_sim::dynamics {

namespace {
    inline constexpr double kMinSpeedThreshold { 0.05 };
} // namespace

EarthRoverDynamics::EarthRoverDynamics() noexcept
    : EarthRoverDynamics(EarthRoverConfig { })
{
}

EarthRoverDynamics::EarthRoverDynamics(const EarthRoverConfig& config) noexcept
    : config_ { config }
{
    reset();
}

void EarthRoverDynamics::step(physics::RigidBody2D& body, double dt) noexcept
{
    if (dt <= 0.0) {
        return;
    }

    if (input_.jump) {
        triggerJump();
    }

    resolveSuspension(body, dt);

    if (jumpTriggered_) {
        if (isGrounded()) {
            const double jumpV { (jumpSpeedOverride_ > 0.0) ? jumpSpeedOverride_ : config_.jumpSpeed };
            const double impulse { (config_.jumpImpulse > 0.0) ? config_.jumpImpulse : (config_.mass * jumpV) };
            body.applyLinearImpulse(math::Vec2 { 0.0, impulse });

            for (auto& wheel : wheelStates_) {
                wheel.isGrounded = false;
                wheel.normalForce = 0.0;
            }
            isAirborne_ = true;
        }
        jumpTriggered_ = false;
        jumpSpeedOverride_ = 0.0;
    }

    applyTraction(body);
    applyAeroDrag(body);
    applyAttitudeStabilizer(body);
}

void EarthRoverDynamics::reset() noexcept
{
    forwardSpeed_ = 0.0;
    isAirborne_ = true;
    jumpTriggered_ = false;
    jumpSpeedOverride_ = 0.0;
    input_ = EarthRoverInput { };

    for (auto& wheel : wheelStates_) {
        wheel.worldPos = math::Vec2::zero();
        wheel.contactPoint = math::Vec2::zero();
        wheel.surfaceNormal = math::Vec2 { 0.0, 1.0 };
        wheel.compression = 0.0;
        wheel.normalForce = 0.0;
        wheel.tractionForce = 0.0;
        wheel.isGrounded = false;
    }
}

bool EarthRoverDynamics::isAirborne() const noexcept
{
    return isAirborne_;
}

bool EarthRoverDynamics::isGrounded() const noexcept
{
    return !isAirborne_;
}

void EarthRoverDynamics::setInput(const EarthRoverInput& input) noexcept
{
    input_ = input;
}

const EarthRoverInput& EarthRoverDynamics::getInput() const noexcept
{
    return input_;
}

void EarthRoverDynamics::setTerrain(const terrain::Heightfield1D* terrain) noexcept
{
    terrain_ = terrain;
}

const terrain::Heightfield1D* EarthRoverDynamics::getTerrain() const noexcept
{
    return terrain_;
}

void EarthRoverDynamics::triggerJump(double speed) noexcept
{
    jumpTriggered_ = true;
    jumpSpeedOverride_ = speed;
}

const EarthRoverWheelState& EarthRoverDynamics::getWheelState(std::size_t index) const noexcept
{
    return wheelStates_.at(index);
}

const EarthRoverConfig& EarthRoverDynamics::getConfig() const noexcept
{
    return config_;
}

void EarthRoverDynamics::setConfig(const EarthRoverConfig& config) noexcept
{
    config_ = config;
}

double EarthRoverDynamics::getSpeed() const noexcept
{
    return forwardSpeed_;
}

void EarthRoverDynamics::resolveSuspension(physics::RigidBody2D& body, double dt) noexcept
{
    static_cast<void>(dt);

    const double theta { body.getOrientation() };
    const double cosTheta { std::cos(theta) };
    const double sinTheta { std::sin(theta) };

    const math::Vec2 forwardDir { cosTheta, sinTheta };
    const math::Vec2 upwardDir { -sinTheta, cosTheta };
    const math::Vec2 downwardDir { sinTheta, -cosTheta };

    body.applyForce(math::Vec2 { 0.0, -config_.mass * config_.gravity });

    std::size_t groundedCount { 0 };

    for (std::size_t i { 0 }; i < kWheelCount; ++i) {
        auto& wheel { wheelStates_[i] };
        const auto& wheelConfig { config_.wheels[i] };

        const math::Vec2 attachLocal { forwardDir * wheelConfig.longitudinalOffset };
        const math::Vec2 attachWorld { body.toWorldPoint(attachLocal) };
        const math::Vec2 attachVel { body.getVelocityAtPoint(attachWorld) };

        const math::Vec2 uncompressedHub { attachWorld + (downwardDir * wheelConfig.restLength) };
        const double xQuery { uncompressedHub.x };

        double groundY { 0.0 };
        math::Vec2 groundNormal { 0.0, 1.0 };
        if (terrain_ != nullptr) {
            const auto sample { terrain_->sampleSurface(xQuery) };
            groundY = sample.elevation;
            groundNormal = sample.normal;
        }

        const double contactElevation { groundY + wheelConfig.wheelRadius };
        const double penetration { contactElevation - uncompressedHub.y };

        if (penetration > 0.0) {
            const double maxAllowedCompression { wheelConfig.restLength * 0.95 };
            const double compression { std::min(penetration, maxAllowedCompression) };
            const math::Vec2 hubPos { uncompressedHub + (upwardDir * compression) };

            const double velAlongStrut { attachVel.dot(upwardDir) };
            const double springForce { wheelConfig.springStiffness * compression };
            const double dampingForce { -wheelConfig.dampingCoeff * velAlongStrut };
            const double normalMagnitude { std::max(0.0, springForce + dampingForce) };

            const math::Vec2 normalForceVec { groundNormal * normalMagnitude };
            body.applyForceAtPoint(normalForceVec, hubPos);

            wheel.worldPos = hubPos;
            wheel.contactPoint = math::Vec2 { xQuery, groundY };
            wheel.surfaceNormal = groundNormal;
            wheel.compression = compression;
            wheel.normalForce = normalMagnitude;
            wheel.isGrounded = true;
            ++groundedCount;
        } else {
            wheel.worldPos = uncompressedHub;
            wheel.contactPoint = math::Vec2 { xQuery, groundY };
            wheel.surfaceNormal = groundNormal;
            wheel.compression = 0.0;
            wheel.normalForce = 0.0;
            wheel.tractionForce = 0.0;
            wheel.isGrounded = false;
        }
    }

    isAirborne_ = (groundedCount == 0);
}

void EarthRoverDynamics::applyTraction(physics::RigidBody2D& body) noexcept
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
    for (const auto& wheel : wheelStates_) {
        if (wheel.isGrounded) {
            totalNormalForce += wheel.normalForce;
        }
    }

    if (totalNormalForce <= 1e-4) {
        return;
    }

    for (std::size_t i { 0 }; i < kWheelCount; ++i) {
        auto& wheel { wheelStates_[i] };
        if (!wheel.isGrounded) {
            wheel.tractionForce = 0.0;
            continue;
        }

        double frictionCoeff { config_.groundFriction };
        if (terrain_ != nullptr) {
            frictionCoeff = terrain_->getMaterial(wheel.contactPoint.x).friction;
        }

        const double wheelShare { wheel.normalForce / totalNormalForce };
        const double wheelDemand { totalDemand * wheelShare };
        const double maxTraction { frictionCoeff * wheel.normalForce };
        const double actualTraction { std::clamp(wheelDemand, -maxTraction, maxTraction) };

        math::Vec2 surfaceTangent { wheel.surfaceNormal.y, -wheel.surfaceNormal.x };
        if (surfaceTangent.dot(forwardDir) < 0.0) {
            surfaceTangent = math::Vec2 { -surfaceTangent.x, -surfaceTangent.y };
        }

        const math::Vec2 tractionForceVec { surfaceTangent * actualTraction };
        body.applyForceAtPoint(tractionForceVec, wheel.worldPos);
        wheel.tractionForce = actualTraction;
    }
}

void EarthRoverDynamics::applyAeroDrag(physics::RigidBody2D& body) noexcept
{
    const math::Vec2 vel { body.getLinearVelocity() };
    const double speedSq { vel.lengthSquared() };
    if (speedSq <= 1e-6) {
        return;
    }

    const double speed { std::sqrt(speedSq) };
    const double dragMag { 0.5 * config_.airDensity * config_.dragCoefficient * config_.frontalArea * speedSq };
    const math::Vec2 dragForce { (vel / speed) * (-dragMag) };

    body.applyForce(dragForce);
}

void EarthRoverDynamics::applyAttitudeStabilizer(physics::RigidBody2D& body) noexcept
{
    if (!isAirborne_) {
        return;
    }

    const double orientation { body.getOrientation() };
    const double normalizedAngle { physics::Integrator2D::normalizeAngle(orientation) };
    const double angleError { -normalizedAngle };

    const double angularVel { body.getAngularVelocity() };
    const double restoringTorque { (angleError * config_.pitchStabGain) - (angularVel * config_.pitchDampGain) };

    body.applyTorque(restoringTorque);
}

} // namespace vehicle_sim::dynamics
