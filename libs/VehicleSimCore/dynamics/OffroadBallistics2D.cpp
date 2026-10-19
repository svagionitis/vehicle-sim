#include "OffroadBallistics2D.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

#include "libs/VehicleSimCore/physics/RigidBody2D.h"

namespace vehicle_sim::dynamics {

namespace {
    inline constexpr double kAirDensity { 1.225 };
    inline constexpr double kGravityY { -9.81 };
    inline constexpr double kMortarAngleRad { 1.1344640137963142 }; // ~65 degrees
} // namespace

OffroadBallistics2D::OffroadBallistics2D() noexcept
{
    // Nominal Rocket Configuration
    rocketConfig_.mass = 6.0;
    rocketConfig_.muzzleVelocity = 48.0;
    rocketConfig_.launchAngleOffset = 0.0;
    rocketConfig_.dragCoefficient = 0.22;
    rocketConfig_.crossSectionArea = 0.015;
    rocketConfig_.blastRadius = 2.5;
    rocketConfig_.blastMaxImpulse = 4200.0;
    rocketConfig_.baseDamage = 50.0;
    rocketConfig_.maxLifespan = 3.5;

    // Nominal Mortar Configuration (higher trajectory, wider blast radius)
    mortarConfig_.mass = 12.0;
    mortarConfig_.muzzleVelocity = 26.0;
    mortarConfig_.launchAngleOffset = kMortarAngleRad;
    mortarConfig_.dragCoefficient = 0.35;
    mortarConfig_.crossSectionArea = 0.025;
    mortarConfig_.blastRadius = 3.8;
    mortarConfig_.blastMaxImpulse = 6000.0;
    mortarConfig_.baseDamage = 85.0;
    mortarConfig_.maxLifespan = 5.0;
}

void OffroadBallistics2D::step(double dt, const terrain::Heightfield1D* terrain) noexcept
{
    if (dt <= 0.0) {
        return;
    }

    for (auto& proj : projectiles_) {
        if (!proj.isAlive) {
            continue;
        }

        proj.age += dt;
        const auto& cfg { (proj.type == ProjectileType::ForwardRocket) ? rocketConfig_ : mortarConfig_ };

        if (proj.age >= cfg.maxLifespan) {
            proj.isAlive = false;
            pendingBlasts_.push_back(BlastEvent { proj.position, cfg.blastRadius, cfg.baseDamage, cfg.blastMaxImpulse });
            continue;
        }

        const double speedSq { proj.velocity.lengthSquared() };
        const double speed { std::sqrt(speedSq) };

        math::Vec2 aeroAccel { 0.0, 0.0 };
        if (speed > 1e-4) {
            const double dragMag { 0.5 * kAirDensity * cfg.dragCoefficient * cfg.crossSectionArea * speedSq };
            aeroAccel = proj.velocity * (-dragMag / (cfg.mass * speed));
        }

        const math::Vec2 accel { aeroAccel.x, aeroAccel.y + kGravityY };

        // Symplectic Euler integration
        proj.velocity += accel * dt;
        proj.position += proj.velocity * dt;
        proj.heading = std::atan2(proj.velocity.y, proj.velocity.x);

        // Continuous ground collision check
        if (terrain != nullptr) {
            const auto sample { terrain->sampleSurface(proj.position.x) };
            if (proj.position.y <= sample.elevation) {
                proj.isAlive = false;
                const math::Vec2 impactPoint { proj.position.x, sample.elevation };
                pendingBlasts_.push_back(BlastEvent { impactPoint, cfg.blastRadius, cfg.baseDamage, cfg.blastMaxImpulse });
            }
        }
    }

    // Prune dead projectiles
    projectiles_.erase(
        std::remove_if(projectiles_.begin(), projectiles_.end(), [](const ProjectileState& p) noexcept {
            return !p.isAlive;
        }),
        projectiles_.end()
    );
}

void OffroadBallistics2D::fireRocket(
    const math::Vec2& origin, double heading, const math::Vec2& baseVelocity) noexcept
{
    const double totalAngle { heading + rocketConfig_.launchAngleOffset };
    const math::Vec2 muzzleDir { std::cos(totalAngle), std::sin(totalAngle) };
    const math::Vec2 launchVel { baseVelocity + (muzzleDir * rocketConfig_.muzzleVelocity) };

    ProjectileState proj {};
    proj.type = ProjectileType::ForwardRocket;
    proj.position = origin;
    proj.velocity = launchVel;
    proj.heading = totalAngle;
    proj.age = 0.0;
    proj.isAlive = true;

    projectiles_.push_back(proj);
}

void OffroadBallistics2D::fireMortar(
    const math::Vec2& origin, const math::Vec2& baseVelocity) noexcept
{
    const double angle { mortarConfig_.launchAngleOffset };
    const math::Vec2 muzzleDir { std::cos(angle), std::sin(angle) };
    const math::Vec2 launchVel { baseVelocity + (muzzleDir * mortarConfig_.muzzleVelocity) };

    ProjectileState proj {};
    proj.type = ProjectileType::AntiAirMortar;
    proj.position = origin;
    proj.velocity = launchVel;
    proj.heading = angle;
    proj.age = 0.0;
    proj.isAlive = true;

    projectiles_.push_back(proj);
}

void OffroadBallistics2D::applyBlast(
    const BlastEvent& blast, const std::vector<physics::RigidBody2D*>& bodies) noexcept
{
    if (blast.radius <= 0.0 || blast.maxImpulse <= 0.0) {
        return;
    }

    for (auto* body : bodies) {
        if (body == nullptr) {
            continue;
        }

        const math::Vec2 bodyPos { body->getPosition() };
        const math::Vec2 toBody { bodyPos - blast.epicenter };
        const double distSq { toBody.lengthSquared() };

        if (distSq >= (blast.radius * blast.radius)) {
            continue;
        }

        const double dist { std::sqrt(distSq) };
        const double falloff { 1.0 - (dist / blast.radius) };
        const double impulseMag { blast.maxImpulse * falloff * falloff };

        math::Vec2 impulseDir { 0.0, 1.0 };
        if (dist > 1e-4) {
            impulseDir = toBody * (1.0 / dist);
        }

        const math::Vec2 impulse { impulseDir * impulseMag };
        body->applyLinearImpulse(impulse);
    }
}

void OffroadBallistics2D::detonateAt(std::size_t index, const math::Vec2& impactPoint) noexcept
{
    if (index >= projectiles_.size()) {
        return;
    }

    auto& proj { projectiles_[index] };
    if (!proj.isAlive) {
        return;
    }

    proj.isAlive = false;
    proj.position = impactPoint;
    const auto& cfg { (proj.type == ProjectileType::ForwardRocket) ? rocketConfig_ : mortarConfig_ };
    pendingBlasts_.push_back(BlastEvent { impactPoint, cfg.blastRadius, cfg.baseDamage, cfg.blastMaxImpulse });
}

void OffroadBallistics2D::detonate(std::size_t index) noexcept
{
    if (index >= projectiles_.size()) {
        return;
    }
    detonateAt(index, projectiles_[index].position);
}

const std::vector<ProjectileState>& OffroadBallistics2D::getProjectiles() const noexcept
{
    return projectiles_;
}

std::vector<BlastEvent> OffroadBallistics2D::consumeBlasts() noexcept
{
    std::vector<BlastEvent> blasts { std::move(pendingBlasts_) };
    pendingBlasts_.clear();
    return blasts;
}

std::size_t OffroadBallistics2D::getActiveCount() const noexcept
{
    return projectiles_.size();
}

void OffroadBallistics2D::reset() noexcept
{
    projectiles_.clear();
    pendingBlasts_.clear();
}

const ProjectileConfig& OffroadBallistics2D::getRocketConfig() const noexcept
{
    return rocketConfig_;
}

void OffroadBallistics2D::setRocketConfig(const ProjectileConfig& config) noexcept
{
    rocketConfig_ = config;
}

const ProjectileConfig& OffroadBallistics2D::getMortarConfig() const noexcept
{
    return mortarConfig_;
}

void OffroadBallistics2D::setMortarConfig(const ProjectileConfig& config) noexcept
{
    mortarConfig_ = config;
}

} // namespace vehicle_sim::dynamics
