#include "SideViewCarDynamics.h"

#include <algorithm>
#include <cmath>

#include "libs/VehicleSimCore/math/Vec2.h"
#include "libs/VehicleSimCore/physics/ContactSolver2D.h"
#include "libs/VehicleSimCore/physics/Integrator2D.h"
#include "libs/VehicleSimCore/physics/RigidBody2D.h"

namespace vehicle_sim::dynamics {

inline constexpr double kPi { 3.14159265358979323846 };
inline constexpr double kGroundTolerance { 0.03 };
inline constexpr double kUprightSpring { 15000.0 };
inline constexpr double kAngularDamp { 1200.0 };
inline constexpr double kPenetrationStiffness { 25000.0 };
inline constexpr double kPenetrationDamping { 600.0 };

SideViewCarDynamics::SideViewCarDynamics() noexcept
    : SideViewCarDynamics(SideViewCarConfig { })
{
}

SideViewCarDynamics::SideViewCarDynamics(const SideViewCarConfig& config) noexcept
    : mConfig { config }
{
    updateWheelLayout();
    mPlatforms.push_back(SideViewPlatform { 0.0, -1e6, 1e6, 1.0, false });
}

void SideViewCarDynamics::step(physics::RigidBody2D& body, double dt) noexcept
{
    if (dt <= 0.0) {
        return;
    }

    if (mInput.jump) {
        triggerJump();
    }
    if (mInput.turnaround) {
        triggerTurnaround(false);
    }

    resolvePlatformContact(body, dt);

    if (mJumpTriggered) {
        if (isGrounded()) {
            const double jumpV { (mJumpSpeedOverride > 0.0) ? mJumpSpeedOverride : mConfig.jumpSpeed };
            const double impulse { (mConfig.jumpImpulse > 0.0) ? mConfig.jumpImpulse : (mConfig.mass * jumpV) };
            body.applyLinearImpulse(math::Vec2 { 0.0, impulse });

            for (auto& wheel : mWheels) {
                wheel.isGrounded = false;
                wheel.normalForce = 0.0;
            }
        }
        mJumpTriggered = false;
        mJumpSpeedOverride = 0.0;
    }

    updateTurnaround(body, dt);
    updateOilSlick(body, dt);

    if (isGrounded() && mTurnaroundState == TurnaroundState::Idle && mOilSlickState == OilSlickState::Normal) {
        const double targetAngle { (mFacingDirection == FacingDirection::Left) ? kPi : 0.0 };
        const double angleError { physics::Integrator2D::normalizeAngle(targetAngle - body.getOrientation()) };
        const double restoringTorque { (angleError * kUprightSpring) - (body.getAngularVelocity() * kAngularDamp) };
        body.applyTorque(restoringTorque);
    }

    applyDriveForces(body);
}

void SideViewCarDynamics::reset() noexcept
{
    mFacingDirection = FacingDirection::Right;
    mTurnaroundState = TurnaroundState::Idle;
    mOilSlickState = OilSlickState::Normal;
    mTurnaroundTimer = 0.0;
    mOilSlickTimer = 0.0;
    mSpeed = 0.0;
    mJumpTriggered = false;
    mJumpSpeedOverride = 0.0;
    mTurnaroundTriggered = false;
    mInstantTurnaround = false;

    for (auto& wheel : mWheels) {
        wheel.worldPosition = math::Vec2::zero();
        wheel.velocity = math::Vec2::zero();
        wheel.normalForce = 0.0;
        wheel.tractionForce = 0.0;
        wheel.isGrounded = false;
    }
}

bool SideViewCarDynamics::isAirborne() const noexcept
{
    return !isGrounded();
}

bool SideViewCarDynamics::isGrounded() const noexcept
{
    return mWheels[kRearWheel].isGrounded || mWheels[kFrontWheel].isGrounded;
}

bool SideViewCarDynamics::isSpinningOut() const noexcept
{
    return mOilSlickState == OilSlickState::Spinning;
}

bool SideViewCarDynamics::isTurningAround() const noexcept
{
    return mTurnaroundState == TurnaroundState::Active;
}

FacingDirection SideViewCarDynamics::getFacingDirection() const noexcept
{
    return mFacingDirection;
}

void SideViewCarDynamics::setFacingDirection(FacingDirection direction) noexcept
{
    mFacingDirection = direction;
}

TurnaroundState SideViewCarDynamics::getTurnaroundState() const noexcept
{
    return mTurnaroundState;
}

OilSlickState SideViewCarDynamics::getOilSlickState() const noexcept
{
    return mOilSlickState;
}

double SideViewCarDynamics::getOilSlickTimer() const noexcept
{
    return mOilSlickTimer;
}

double SideViewCarDynamics::getTurnaroundTimer() const noexcept
{
    return mTurnaroundTimer;
}

double SideViewCarDynamics::getSpeed() const noexcept
{
    return mSpeed;
}

void SideViewCarDynamics::setInput(const SideViewCarInput& input) noexcept
{
    mInput = input;
}

const SideViewCarInput& SideViewCarDynamics::getInput() const noexcept
{
    return mInput;
}

const SideViewCarConfig& SideViewCarDynamics::getConfig() const noexcept
{
    return mConfig;
}

void SideViewCarDynamics::setConfig(const SideViewCarConfig& config) noexcept
{
    mConfig = config;
    updateWheelLayout();
}

void SideViewCarDynamics::triggerJump(double speed) noexcept
{
    mJumpTriggered = true;
    mJumpSpeedOverride = speed;
}

void SideViewCarDynamics::triggerTurnaround(bool instant) noexcept
{
    mTurnaroundTriggered = true;
    mInstantTurnaround = instant;
}

void SideViewCarDynamics::triggerOilSlick(double duration) noexcept
{
    mOilSlickState = OilSlickState::Spinning;
    mOilSlickTimer = (duration > 0.0) ? duration : mConfig.oilSlickDuration;
}

void SideViewCarDynamics::addPlatform(const SideViewPlatform& platform)
{
    mPlatforms.push_back(platform);
}

void SideViewCarDynamics::addPlatform(double y, double minX, double maxX, double friction, bool oneWay)
{
    mPlatforms.push_back(SideViewPlatform { y, minX, maxX, friction, oneWay });
}

void SideViewCarDynamics::clearPlatforms() noexcept
{
    mPlatforms.clear();
}

std::size_t SideViewCarDynamics::getPlatformCount() const noexcept
{
    return mPlatforms.size();
}

const std::vector<SideViewPlatform>& SideViewCarDynamics::getPlatforms() const noexcept
{
    return mPlatforms;
}

const SideViewWheelTelemetry& SideViewCarDynamics::getWheelTelemetry(std::size_t index) const noexcept
{
    if (index >= kWheelCount) {
        return mWheels[0];
    }
    return mWheels[index];
}

void SideViewCarDynamics::setSurfaceModifier(double gripMultiplier) noexcept
{
    mGlobalGripModifier = std::max(0.0, gripMultiplier);
}

double SideViewCarDynamics::getSurfaceModifier() const noexcept
{
    return mGlobalGripModifier;
}

void SideViewCarDynamics::updateWheelLayout() noexcept
{
    const double halfWheelbase { mConfig.wheelbase * 0.5 };
    const double radius { mConfig.wheelRadius };

    mWheels[kRearWheel].localOffset = math::Vec2 { -halfWheelbase, -radius };
    mWheels[kFrontWheel].localOffset = math::Vec2 { halfWheelbase, -radius };
}

void SideViewCarDynamics::resolvePlatformContact(physics::RigidBody2D& body, double dt) noexcept
{
    static_cast<void>(dt);
    body.applyForce(math::Vec2 { 0.0, -mConfig.mass * mConfig.gravity });

    for (std::size_t i { 0 }; i < kWheelCount; ++i) {
        SideViewWheelTelemetry& wheel { mWheels[i] };
        wheel.worldPosition = body.toWorldPoint(wheel.localOffset);
        wheel.velocity = body.getVelocityAtPoint(wheel.worldPosition);
        wheel.isGrounded = false;
        wheel.normalForce = 0.0;
        wheel.tractionForce = 0.0;

        const SideViewPlatform* bestPlat { nullptr };
        double highestY { -1e9 };

        for (const auto& plat : mPlatforms) {
            if (wheel.worldPosition.x < plat.minX || wheel.worldPosition.x > plat.maxX) {
                continue;
            }

            if (plat.oneWay && wheel.velocity.y > 0.05) {
                continue;
            }

            if (wheel.worldPosition.y <= plat.y + kGroundTolerance && wheel.worldPosition.y >= plat.y - 0.40) {
                if (plat.y > highestY) {
                    highestY = plat.y;
                    bestPlat = &plat;
                }
            }
        }

        if (bestPlat != nullptr) {
            if (wheel.velocity.y < -1e-4) {
                physics::Contact2D contact { };
                contact.point = math::Vec2 { wheel.worldPosition.x, bestPlat->y };
                contact.normal = math::Vec2 { 0.0, -1.0 };
                contact.penetration = std::max(0.0, bestPlat->y - wheel.worldPosition.y);
                contact.restitution = mConfig.groundRestitution;
                contact.friction = mConfig.platformFriction * bestPlat->friction;

                static_cast<void>(physics::ContactSolver2D::resolve(body, nullptr, contact));
                wheel.velocity = body.getVelocityAtPoint(wheel.worldPosition);
            }

            wheel.isGrounded = true;
            double normalF { (mConfig.mass * mConfig.gravity) * 0.5 };
            const double penetration { bestPlat->y - wheel.worldPosition.y };
            if (penetration > 0.0) {
                normalF
                    += (kPenetrationStiffness * penetration) - (kPenetrationDamping * std::min(0.0, wheel.velocity.y));
            }

            wheel.normalForce = normalF;
            body.applyForceAtPoint(math::Vec2 { 0.0, normalF }, wheel.worldPosition);
        }
    }
}

void SideViewCarDynamics::updateTurnaround(physics::RigidBody2D& body, double dt) noexcept
{
    if (mTurnaroundTriggered) {
        const FacingDirection targetDir { (mFacingDirection == FacingDirection::Right) ? FacingDirection::Left
                                                                                       : FacingDirection::Right };
        if (mInstantTurnaround || mConfig.turnaroundDuration <= 0.0) {
            mFacingDirection = targetDir;
            body.setOrientation((targetDir == FacingDirection::Left) ? kPi : 0.0);
            body.setAngularVelocity(0.0);
            mTurnaroundState = TurnaroundState::Idle;
            mTurnaroundTimer = 0.0;
        } else {
            mFacingDirection = targetDir;
            mTurnaroundState = TurnaroundState::Active;
            mTurnaroundTimer = mConfig.turnaroundDuration;
        }
        mTurnaroundTriggered = false;
        mInstantTurnaround = false;
    }

    if (mTurnaroundState == TurnaroundState::Active) {
        const double targetAngle { (mFacingDirection == FacingDirection::Left) ? kPi : 0.0 };
        const double currentAngle { body.getOrientation() };
        const double diff { physics::Integrator2D::normalizeAngle(targetAngle - currentAngle) };

        if (mTurnaroundTimer <= dt || std::abs(diff) < 1e-3) {
            body.setOrientation(targetAngle);
            body.setAngularVelocity(0.0);
            mTurnaroundState = TurnaroundState::Idle;
            mTurnaroundTimer = 0.0;
        } else {
            const double omega { diff / mTurnaroundTimer };
            body.setAngularVelocity(omega);
            mTurnaroundTimer -= dt;
        }
    }
}

void SideViewCarDynamics::updateOilSlick(physics::RigidBody2D& body, double dt) noexcept
{
    if (mOilSlickState == OilSlickState::Spinning) {
        body.applyTorque(mConfig.oilSlickTorque);
        mOilSlickTimer -= dt;

        if (mOilSlickTimer <= 0.0) {
            mOilSlickState = OilSlickState::Normal;
            mOilSlickTimer = 0.0;
            const double targetAngle { (mFacingDirection == FacingDirection::Left) ? kPi : 0.0 };
            body.setOrientation(targetAngle);
            body.setAngularVelocity(0.0);
        }
    }
}

void SideViewCarDynamics::applyDriveForces(physics::RigidBody2D& body) noexcept
{
    const math::Vec2 fwdLocal { 1.0, 0.0 };
    const math::Vec2 fwdWorld { body.toWorldVec(fwdLocal) };
    mSpeed = body.getLinearVelocity().dot(fwdWorld);

    const math::Vec2 vel { body.getLinearVelocity() };
    const double speedMag { vel.length() };
    if (speedMag > 1e-4) {
        body.applyForce(-mConfig.linearDrag * speedMag * vel);
    }

    if (isAirborne()) {
        return;
    }

    if (mOilSlickState == OilSlickState::Spinning) {
        const double residualFriction { mGlobalGripModifier * mConfig.oilSlickFriction };
        if (speedMag > 1e-4) {
            body.applyForce(-mConfig.rollingResistance * residualFriction * vel);
        }
        return;
    }

    const double grip { mGlobalGripModifier };
    double driveForce { 0.0 };

    if (mInput.throttle > 0.0) {
        driveForce = std::clamp(mInput.throttle, 0.0, 1.0) * mConfig.maxDriveForce;
    } else if (mInput.throttle < 0.0) {
        driveForce = std::clamp(mInput.throttle, -1.0, 0.0) * mConfig.maxReverseForce;
    }
    driveForce *= grip;

    if (mInput.brake > 0.0 && std::abs(mSpeed) > 0.01) {
        const double brakeForce { std::clamp(mInput.brake, 0.0, 1.0) * mConfig.maxBrakeForce * grip };
        if (mSpeed > 0.0) {
            driveForce -= brakeForce;
        } else {
            driveForce += brakeForce;
        }
    }

    const double rrForce { mConfig.rollingResistance * mSpeed * grip };
    driveForce -= rrForce;

    std::size_t groundedCount { 0 };
    for (const auto& wheel : mWheels) {
        if (wheel.isGrounded) {
            ++groundedCount;
        }
    }

    if (groundedCount > 0) {
        const double forcePerWheel { driveForce / static_cast<double>(groundedCount) };
        const math::Vec2 forceVec { fwdWorld * forcePerWheel };
        for (auto& wheel : mWheels) {
            if (wheel.isGrounded) {
                body.applyForceAtPoint(forceVec, wheel.worldPosition);
                wheel.tractionForce = forcePerWheel;
            }
        }
    }
}

} // namespace vehicle_sim::dynamics
