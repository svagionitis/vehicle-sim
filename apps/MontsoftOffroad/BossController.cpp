#include "apps/MontsoftOffroad/BossController.h"

#include <algorithm>
#include <cmath>

namespace montsoft_offroad {

using vehicle_sim::math::Vec2;

BossController::BossController() noexcept
    : BossController(BossControllerConfig{})
{
}

BossController::BossController(const BossControllerConfig& config) noexcept
    : config_{ config }
{
    reset();
}

void BossController::reset() noexcept
{
    currentPhase_ = BossPhase::Inactive;
    mechAttack_ = MechHeadAttack::Idle;
    phaseTimer_ = 0.0;
    attackTimer_ = 1.0;
    flakTimer_ = 2.0;
    deathExplosionTimer_ = 0.0;
    hoverTime_ = 0.0;
    arenaLocked_ = false;
    attackToggle_ = false;

    mechHeadPos_ = Vec2{ 480.0, 0.0 };
    cruiserPos_ = Vec2{ 460.0, 22.0 };
    cruiserVelocity_ = Vec2{ 0.0, 0.0 };

    // Initialize Mech Head components
    mechMouth_.targetId = kTargetMechMouth;
    mechMouth_.relativeOffset = Vec2{ -2.0, 2.4 };
    mechMouth_.dimensions = Vec2{ 3.2, 2.0 };
    mechMouth_.maxHealth = config_.mechHeadMaxHealth * 0.5;
    mechMouth_.health = mechMouth_.maxHealth;
    mechMouth_.flashTimer = 0.0;
    mechMouth_.isExposed = false;
    mechMouth_.isDestroyed = false;

    mechEyes_.targetId = kTargetMechEyes;
    mechEyes_.relativeOffset = Vec2{ -1.2, 5.8 };
    mechEyes_.dimensions = Vec2{ 2.8, 1.6 };
    mechEyes_.maxHealth = config_.mechHeadMaxHealth * 0.5;
    mechEyes_.health = mechEyes_.maxHealth;
    mechEyes_.flashTimer = 0.0;
    mechEyes_.isExposed = false;
    mechEyes_.isDestroyed = false;

    // Initialize Dreadnought Cruiser components
    cruiserPort_.targetId = kTargetCruiserPort;
    cruiserPort_.relativeOffset = Vec2{ -5.0, -1.2 };
    cruiserPort_.dimensions = Vec2{ 3.2, 2.0 };
    cruiserPort_.maxHealth = config_.cruiserTurretHealth;
    cruiserPort_.health = cruiserPort_.maxHealth;
    cruiserPort_.flashTimer = 0.0;
    cruiserPort_.isExposed = true;
    cruiserPort_.isDestroyed = false;

    cruiserStarboard_.targetId = kTargetCruiserStarboard;
    cruiserStarboard_.relativeOffset = Vec2{ 5.0, -1.2 };
    cruiserStarboard_.dimensions = Vec2{ 3.2, 2.0 };
    cruiserStarboard_.maxHealth = config_.cruiserTurretHealth;
    cruiserStarboard_.health = cruiserStarboard_.maxHealth;
    cruiserStarboard_.flashTimer = 0.0;
    cruiserStarboard_.isExposed = true;
    cruiserStarboard_.isDestroyed = false;

    cruiserCore_.targetId = kTargetCruiserCore;
    cruiserCore_.relativeOffset = Vec2{ 0.0, 0.2 };
    cruiserCore_.dimensions = Vec2{ 3.6, 2.6 };
    cruiserCore_.maxHealth = config_.cruiserCoreHealth;
    cruiserCore_.health = cruiserCore_.maxHealth;
    cruiserCore_.flashTimer = 0.0;
    cruiserCore_.isExposed = false; // Protected by force barrier initially
    cruiserCore_.isDestroyed = false;

    for (auto& bomb : clusterBombs_) {
        bomb.active = false;
    }
    for (auto& beam : eyeLasers_) {
        beam.active = false;
    }
    for (auto& flak : flakShells_) {
        flak.active = false;
    }
}

void BossController::triggerBossEncounter(BossPhase phase) noexcept
{
    currentPhase_ = phase;
    phaseTimer_ = 0.0;
    attackTimer_ = 1.0;
    arenaLocked_ = true;

    if (phase == BossPhase::MechHeadCombat) {
        mechHeadPos_.y = 0.0;
    } else if (phase == BossPhase::CruiserCombat) {
        cruiserPos_ = Vec2{ 460.0, 9.5 };
    }
}

void BossController::update(
    double dt,
    const vehicle_sim::terrain::Heightfield1D* terrain,
    Camera2D& camera,
    vehicle_sim::physics::RigidBody2D& truckBody,
    ParticleSystem& particleSystem
) noexcept
{
    if (dt <= 0.0) {
        return;
    }

    // Auto-trigger when entering Stage 8 arena
    if (currentPhase_ == BossPhase::Inactive) {
        if (truckBody.getPosition().x >= config_.arenaTriggerX) {
            triggerBossEncounter(BossPhase::MechHeadEntrance);
        }
    }

    if (!arenaLocked_) {
        return;
    }

    phaseTimer_ += dt;

    // Enforce arena restitution and camera clamping
    enforceArenaBounds(truckBody);
    camera.clampPositionX(config_.cameraMinX, config_.cameraMaxX);

    switch (currentPhase_) {
    case BossPhase::MechHeadEntrance:
    case BossPhase::MechHeadCombat:
    case BossPhase::MechHeadDefeated:
        updateMechHead(dt, terrain, truckBody, particleSystem, camera);
        break;

    case BossPhase::CruiserDescent:
    case BossPhase::CruiserCombat:
    case BossPhase::CruiserDefeated:
        updateCruiser(dt, truckBody, particleSystem, camera);
        break;

    case BossPhase::VictoryCleared:
    case BossPhase::Inactive:
    default:
        break;
    }
}

void BossController::updateMechHead(
    double dt,
    const vehicle_sim::terrain::Heightfield1D* terrain,
    vehicle_sim::physics::RigidBody2D& truckBody,
    ParticleSystem& particleSystem,
    Camera2D& camera
) noexcept
{
    // 1. Entrance State
    if (currentPhase_ == BossPhase::MechHeadEntrance) {
        constexpr double kEntranceDuration { 2.0 };
        const double progress { std::min(1.0, phaseTimer_ / kEntranceDuration) };
        mechHeadPos_.y = -8.0 + (8.0 * progress);

        // Ground tremor and dust emission
        camera.addTrauma(0.25 * dt);
        particleSystem.emitLandingDust(mechHeadPos_ + Vec2{ -1.5, 0.0 }, 2.5, 7U);
        particleSystem.emitLandingDust(mechHeadPos_ + Vec2{ 1.5, 0.0 }, 2.5, 7U);

        if (phaseTimer_ >= kEntranceDuration) {
            currentPhase_ = BossPhase::MechHeadCombat;
            phaseTimer_ = 0.0;
            mechAttack_ = MechHeadAttack::Idle;
            attackTimer_ = 1.2;
        }
        return;
    }

    // 2. Defeated State
    if (currentPhase_ == BossPhase::MechHeadDefeated) {
        deathExplosionTimer_ += dt;
        camera.addTrauma(0.6 * dt);

        if (deathExplosionTimer_ >= 0.18) {
            deathExplosionTimer_ = 0.0;
            const double ox { ((phaseTimer_ * 17.0) - std::floor(phaseTimer_ * 17.0) - 0.5) * 6.0 };
            const double oy { ((phaseTimer_ * 29.0) - std::floor(phaseTimer_ * 29.0)) * 7.0 };
            particleSystem.emitExplosionShrapnel(mechHeadPos_ + Vec2{ ox, oy }, 3.0, 1.0, 7U);
        }

        constexpr double kDefeatDuration { 2.5 };
        if (phaseTimer_ >= kDefeatDuration) {
            currentPhase_ = BossPhase::CruiserDescent;
            phaseTimer_ = 0.0;
            cruiserPos_ = Vec2{ 460.0, 22.0 };
        }
        return;
    }

    // 3. Active Combat State
    mechMouth_.flashTimer = std::max(0.0, mechMouth_.flashTimer - dt);
    mechEyes_.flashTimer = std::max(0.0, mechEyes_.flashTimer - dt);

    const double totalHeadHp { mechMouth_.health + mechEyes_.health };
    if (totalHeadHp <= 0.0) {
        currentPhase_ = BossPhase::MechHeadDefeated;
        phaseTimer_ = 0.0;
        deathExplosionTimer_ = 0.0;
        mechMouth_.isExposed = false;
        mechEyes_.isExposed = false;
        return;
    }

    attackTimer_ -= dt;

    switch (mechAttack_) {
    case MechHeadAttack::Idle:
        mechMouth_.isExposed = false;
        mechEyes_.isExposed = false;
        if (attackTimer_ <= 0.0) {
            attackToggle_ = !attackToggle_;
            if (attackToggle_) {
                mechAttack_ = MechHeadAttack::MouthOpenExposed;
                attackTimer_ = 0.8;
            } else {
                mechAttack_ = MechHeadAttack::EyeLaserTelegraph;
                attackTimer_ = 0.8;
            }
        }
        break;

    case MechHeadAttack::EyeLaserTelegraph:
        mechEyes_.isExposed = true; // Vulnerable to anti-air mortar!
        mechMouth_.isExposed = false;
        camera.addTrauma(0.12 * dt);
        if (attackTimer_ <= 0.0) {
            fireEyeLasers();
            mechAttack_ = MechHeadAttack::EyeLaserDischarge;
            attackTimer_ = 0.6;
        }
        break;

    case MechHeadAttack::EyeLaserDischarge: {
        mechEyes_.isExposed = true;
        const Vec2 truckPos { truckBody.getPosition() };
        for (const auto& beam : eyeLasers_) {
            if (beam.active) {
                if (truckPos.x >= beam.targetX && truckPos.x <= beam.originX) {
                    if (std::abs(truckPos.y - beam.elevationY) < 1.2) {
                        truckBody.applyLinearImpulse(Vec2{ -350.0 * dt, -150.0 * dt });
                        particleSystem.emitThrusterSparks(truckPos, Vec2{ -10.0, 5.0 });
                    }
                }
            }
        }
        if (attackTimer_ <= 0.0) {
            for (auto& beam : eyeLasers_) {
                beam.active = false;
            }
            mechAttack_ = MechHeadAttack::SonicRoarWave;
            attackTimer_ = 0.6;
        }
        break;
    }

    case MechHeadAttack::MouthOpenExposed:
        mechMouth_.isExposed = true; // Vulnerable to forward rockets!
        mechEyes_.isExposed = false;
        if (attackTimer_ <= 0.0) {
            vomitClusterBombs();
            mechAttack_ = MechHeadAttack::ClusterBombDischarge;
            attackTimer_ = 1.4;
        }
        break;

    case MechHeadAttack::ClusterBombDischarge:
        mechMouth_.isExposed = true;
        if (attackTimer_ <= 0.0) {
            mechAttack_ = MechHeadAttack::SonicRoarWave;
            attackTimer_ = 0.6;
        }
        break;

    case MechHeadAttack::SonicRoarWave:
        mechMouth_.isExposed = false;
        mechEyes_.isExposed = false;
        emitSonicRoar(truckBody, camera);
        mechAttack_ = MechHeadAttack::Idle;
        attackTimer_ = 1.6;
        break;

    default:
        break;
    }

    // Update active rolling cluster bombs
    const Vec2 truckPos { truckBody.getPosition() };
    for (auto& bomb : clusterBombs_) {
        if (!bomb.active) {
            continue;
        }
        bomb.age += dt;
        if (bomb.age >= bomb.maxLifespan) {
            bomb.active = false;
            continue;
        }

        bomb.velocity.x += 1.8 * dt; // Slight deceleration
        bomb.position.x += bomb.velocity.x * dt;

        if (terrain != nullptr) {
            const double groundY { terrain->getHeight(bomb.position.x) + bomb.radius };
            if (bomb.position.y > groundY) {
                bomb.velocity.y -= 9.81 * dt;
                bomb.position.y += bomb.velocity.y * dt;
            } else {
                bomb.position.y = groundY;
                bomb.velocity.y = 0.0;
            }
        }

        // Proximity detonation against player truck
        const double distToTruck { (bomb.position - truckPos).length() };
        if (distToTruck < 2.0) {
            bomb.active = false;
            particleSystem.emitExplosionShrapnel(bomb.position, 2.5, 1.0, 7U);
            camera.addTrauma(0.35);
            truckBody.applyLinearImpulse(Vec2{ -400.0, 2200.0 });
        }
    }
}

void BossController::updateCruiser(
    double dt,
    vehicle_sim::physics::RigidBody2D& truckBody,
    ParticleSystem& particleSystem,
    Camera2D& camera
) noexcept
{
    // 1. Descent Entrance
    if (currentPhase_ == BossPhase::CruiserDescent) {
        constexpr double kDescentDuration { 2.5 };
        const double progress { std::min(1.0, phaseTimer_ / kDescentDuration) };
        cruiserPos_.y = 22.0 - (12.5 * progress);

        // Thruster exhaust particles
        particleSystem.emitThrusterSparks(cruiserPos_ + Vec2{ -6.0, -1.0 }, Vec2{ -8.0, -12.0 });
        particleSystem.emitThrusterSparks(cruiserPos_ + Vec2{  6.0, -1.0 }, Vec2{  8.0, -12.0 });
        camera.addTrauma(0.18 * dt);

        if (phaseTimer_ >= kDescentDuration) {
            currentPhase_ = BossPhase::CruiserCombat;
            phaseTimer_ = 0.0;
            hoverTime_ = 0.0;
            flakTimer_ = 2.0;
        }
        return;
    }

    // 2. Defeated Overload State
    if (currentPhase_ == BossPhase::CruiserDefeated) {
        deathExplosionTimer_ += dt;
        camera.addTrauma(0.8 * dt);

        if (deathExplosionTimer_ >= 0.12) {
            deathExplosionTimer_ = 0.0;
            const double ox { ((phaseTimer_ * 19.0) - std::floor(phaseTimer_ * 19.0) - 0.5) * 12.0 };
            const double oy { ((phaseTimer_ * 31.0) - std::floor(phaseTimer_ * 31.0) - 0.5) * 4.0 };
            particleSystem.emitExplosionShrapnel(cruiserPos_ + Vec2{ ox, oy }, 4.0, 1.2, 7U);
        }

        constexpr double kCruiserDefeatDuration { 3.0 };
        if (phaseTimer_ >= kCruiserDefeatDuration) {
            currentPhase_ = BossPhase::VictoryCleared;
            phaseTimer_ = 0.0;
        }
        return;
    }

    // 3. Active Combat State
    cruiserPort_.flashTimer = std::max(0.0, cruiserPort_.flashTimer - dt);
    cruiserStarboard_.flashTimer = std::max(0.0, cruiserStarboard_.flashTimer - dt);
    cruiserCore_.flashTimer = std::max(0.0, cruiserCore_.flashTimer - dt);

    hoverTime_ += dt;
    cruiserPos_.y = 9.5 + (1.2 * std::sin(0.8 * hoverTime_));

    const double targetX { std::clamp(truckBody.getPosition().x, 442.0, 472.0) };
    cruiserPos_.x += 1.8 * dt * (targetX - cruiserPos_.x);

    particleSystem.emitThrusterSparks(cruiserPos_ + Vec2{ -6.0, -1.0 }, Vec2{ -4.0, -8.0 });
    particleSystem.emitThrusterSparks(cruiserPos_ + Vec2{  6.0, -1.0 }, Vec2{  4.0, -8.0 });

    const bool shieldActive { isShieldActive() };
    cruiserCore_.isExposed = !shieldActive;

    if (cruiserCore_.health <= 0.0) {
        currentPhase_ = BossPhase::CruiserDefeated;
        phaseTimer_ = 0.0;
        deathExplosionTimer_ = 0.0;
        return;
    }

    flakTimer_ -= dt;
    if (flakTimer_ <= 0.0) {
        fireCruiserFlak();
        flakTimer_ = 2.4;
    }

    const Vec2 truckPos { truckBody.getPosition() };
    for (auto& flak : flakShells_) {
        if (!flak.active) {
            continue;
        }
        flak.age += dt;
        if (flak.age >= flak.maxLifespan) {
            flak.active = false;
            continue;
        }

        flak.velocity.y -= 9.81 * dt;
        flak.position += flak.velocity * dt;

        if ((flak.position - truckPos).length() < 1.8) {
            flak.active = false;
            particleSystem.emitExplosionShrapnel(flak.position, 2.0, 1.0, 7U);
            camera.addTrauma(0.3);
            truckBody.applyLinearImpulse(Vec2{ -200.0, -500.0 });
        } else if (flak.position.y <= 0.0) {
            flak.active = false;
            particleSystem.emitExplosionShrapnel(flak.position, 1.8, 0.8, 7U);
        }
    }
}

void BossController::enforceArenaBounds(vehicle_sim::physics::RigidBody2D& truckBody) noexcept
{
    const double truckX { truckBody.getPosition().x };
    constexpr double kStiffness { 120000.0 };
    constexpr double kDamping { 5000.0 };

    if (truckX < config_.arenaMinX) {
        const double deltaX { config_.arenaMinX - truckX };
        const double forceX { (kStiffness * deltaX) - (kDamping * truckBody.getLinearVelocity().x) };
        truckBody.applyForce(Vec2{ std::max(0.0, forceX), 0.0 });
    } else if (truckX > (config_.arenaMaxX - 18.0)) {
        const double limitX { config_.arenaMaxX - 18.0 };
        const double deltaX { truckX - limitX };
        const double forceX { -(kStiffness * deltaX) - (kDamping * truckBody.getLinearVelocity().x) };
        truckBody.applyForce(Vec2{ std::min(0.0, forceX), 0.0 });
    }
}

void BossController::fireEyeLasers() noexcept
{
    eyeLasers_[0].originX = mechHeadPos_.x + mechEyes_.relativeOffset.x;
    eyeLasers_[0].targetX = config_.arenaMinX;
    eyeLasers_[0].elevationY = 2.2;
    eyeLasers_[0].active = true;

    eyeLasers_[1].originX = mechHeadPos_.x + mechEyes_.relativeOffset.x;
    eyeLasers_[1].targetX = config_.arenaMinX;
    eyeLasers_[1].elevationY = 4.2;
    eyeLasers_[1].active = true;
}

void BossController::vomitClusterBombs() noexcept
{
    const Vec2 mouthWorld { mechHeadPos_ + mechMouth_.relativeOffset };
    double launchSpeed { -12.0 };

    for (std::size_t i { 0 }; i < 3; ++i) {
        for (auto& bomb : clusterBombs_) {
            if (!bomb.active) {
                bomb.active = true;
                bomb.age = 0.0;
                bomb.position = mouthWorld;
                bomb.velocity = Vec2{ launchSpeed, 4.5 };
                bomb.radius = 0.55;
                launchSpeed += 2.2;
                break;
            }
        }
    }
}

void BossController::emitSonicRoar(vehicle_sim::physics::RigidBody2D& truckBody, Camera2D& camera) noexcept
{
    camera.addTrauma(0.55);
    truckBody.applyLinearImpulse(Vec2{ -350.0, -2800.0 });
    truckBody.applyAngularImpulse(-1200.0);
}

void BossController::fireCruiserFlak() noexcept
{
    if (cruiserPort_.health > 0.0) {
        for (auto& flak : flakShells_) {
            if (!flak.active) {
                flak.active = true;
                flak.age = 0.0;
                flak.position = cruiserPos_ + cruiserPort_.relativeOffset;
                flak.velocity = Vec2{ -10.0, -4.0 };
                break;
            }
        }
    }
    if (cruiserStarboard_.health > 0.0) {
        for (auto& flak : flakShells_) {
            if (!flak.active) {
                flak.active = true;
                flak.age = 0.0;
                flak.position = cruiserPos_ + cruiserStarboard_.relativeOffset;
                flak.velocity = Vec2{ -16.0, -2.0 };
                break;
            }
        }
    }
}

void BossController::syncCombatTargets(std::vector<CombatTargetAabb>& targets) const noexcept
{
    if (currentPhase_ == BossPhase::MechHeadCombat) {
        if (mechMouth_.isExposed && mechMouth_.health > 0.0) {
            CombatTargetAabb mouthTarget {};
            mouthTarget.targetId = kTargetMechMouth;
            mouthTarget.type = CombatTargetType::BossComponent;
            const Vec2 center { mechHeadPos_ + mechMouth_.relativeOffset };
            mouthTarget.minBounds = center - (mechMouth_.dimensions * 0.5);
            mouthTarget.maxBounds = center + (mechMouth_.dimensions * 0.5);
            mouthTarget.health = mechMouth_.health;
            mouthTarget.isAlive = true;
            targets.push_back(mouthTarget);
        }
        if (mechEyes_.isExposed && mechEyes_.health > 0.0) {
            CombatTargetAabb eyeTarget {};
            eyeTarget.targetId = kTargetMechEyes;
            eyeTarget.type = CombatTargetType::BossComponent;
            const Vec2 center { mechHeadPos_ + mechEyes_.relativeOffset };
            eyeTarget.minBounds = center - (mechEyes_.dimensions * 0.5);
            eyeTarget.maxBounds = center + (mechEyes_.dimensions * 0.5);
            eyeTarget.health = mechEyes_.health;
            eyeTarget.isAlive = true;
            targets.push_back(eyeTarget);
        }
    } else if (currentPhase_ == BossPhase::CruiserCombat) {
        if (cruiserPort_.health > 0.0) {
            CombatTargetAabb portTarget {};
            portTarget.targetId = kTargetCruiserPort;
            portTarget.type = CombatTargetType::BossComponent;
            const Vec2 center { cruiserPos_ + cruiserPort_.relativeOffset };
            portTarget.minBounds = center - (cruiserPort_.dimensions * 0.5);
            portTarget.maxBounds = center + (cruiserPort_.dimensions * 0.5);
            portTarget.health = cruiserPort_.health;
            portTarget.isAlive = true;
            targets.push_back(portTarget);
        }
        if (cruiserStarboard_.health > 0.0) {
            CombatTargetAabb starTarget {};
            starTarget.targetId = kTargetCruiserStarboard;
            starTarget.type = CombatTargetType::BossComponent;
            const Vec2 center { cruiserPos_ + cruiserStarboard_.relativeOffset };
            starTarget.minBounds = center - (cruiserStarboard_.dimensions * 0.5);
            starTarget.maxBounds = center + (cruiserStarboard_.dimensions * 0.5);
            starTarget.health = cruiserStarboard_.health;
            starTarget.isAlive = true;
            targets.push_back(starTarget);
        }
        if (!isShieldActive() && cruiserCore_.health > 0.0) {
            CombatTargetAabb coreTarget {};
            coreTarget.targetId = kTargetCruiserCore;
            coreTarget.type = CombatTargetType::BossComponent;
            const Vec2 center { cruiserPos_ + cruiserCore_.relativeOffset };
            coreTarget.minBounds = center - (cruiserCore_.dimensions * 0.5);
            coreTarget.maxBounds = center + (cruiserCore_.dimensions * 0.5);
            coreTarget.health = cruiserCore_.health;
            coreTarget.isAlive = true;
            targets.push_back(coreTarget);
        }
    }
}

void BossController::applyCombatDamage(
    const std::vector<CombatTargetAabb>& targets,
    ParticleSystem& particleSystem
) noexcept
{
    for (const auto& target : targets) {
        if (target.type != CombatTargetType::BossComponent) {
            continue;
        }

        if (target.targetId == kTargetMechMouth) {
            if (target.health < mechMouth_.health) {
                mechMouth_.health = std::max(0.0, target.health);
                mechMouth_.flashTimer = 0.15;
                particleSystem.emitExplosionShrapnel(mechHeadPos_ + mechMouth_.relativeOffset, 2.2, 1.0, 7U);
            }
        } else if (target.targetId == kTargetMechEyes) {
            if (target.health < mechEyes_.health) {
                mechEyes_.health = std::max(0.0, target.health);
                mechEyes_.flashTimer = 0.15;
                particleSystem.emitExplosionShrapnel(mechHeadPos_ + mechEyes_.relativeOffset, 2.2, 1.0, 7U);
            }
        } else if (target.targetId == kTargetCruiserPort) {
            if (target.health < cruiserPort_.health) {
                cruiserPort_.health = std::max(0.0, target.health);
                cruiserPort_.flashTimer = 0.15;
                particleSystem.emitExplosionShrapnel(cruiserPos_ + cruiserPort_.relativeOffset, 2.5, 1.0, 7U);
            }
        } else if (target.targetId == kTargetCruiserStarboard) {
            if (target.health < cruiserStarboard_.health) {
                cruiserStarboard_.health = std::max(0.0, target.health);
                cruiserStarboard_.flashTimer = 0.15;
                particleSystem.emitExplosionShrapnel(cruiserPos_ + cruiserStarboard_.relativeOffset, 2.5, 1.0, 7U);
            }
        } else if (target.targetId == kTargetCruiserCore) {
            if (target.health < cruiserCore_.health) {
                cruiserCore_.health = std::max(0.0, target.health);
                cruiserCore_.flashTimer = 0.15;
                particleSystem.emitExplosionShrapnel(cruiserPos_ + cruiserCore_.relativeOffset, 3.5, 1.5, 7U);
            }
        }
    }
}

void BossController::render(CgaPaletteRenderer& renderer, const Camera2D& camera) const noexcept
{
    if (currentPhase_ == BossPhase::Inactive) {
        return;
    }

    if (currentPhase_ >= BossPhase::MechHeadEntrance && currentPhase_ <= BossPhase::MechHeadDefeated) {
        renderMechHead(renderer, camera);
        renderBombsAndBeams(renderer, camera);
    } else if (currentPhase_ >= BossPhase::CruiserDescent && currentPhase_ <= BossPhase::CruiserDefeated) {
        renderCruiser(renderer, camera);
        renderBombsAndBeams(renderer, camera);
    }

    renderBossHud(renderer);
}

void BossController::renderMechHead(CgaPaletteRenderer& renderer, const Camera2D& camera) const noexcept
{
    const Vec2 screenPos { camera.worldToScreen(mechHeadPos_) };
    const int sx { static_cast<int>(std::round(screenPos.x)) };
    const int sy { static_cast<int>(std::round(screenPos.y)) };

    const ColorRgba armorColor { 85, 255, 255, 255 };    // Cyan
    const ColorRgba darkColor  { 0, 0, 0, 255 };         // Black
    const ColorRgba highlight  { 255, 255, 255, 255 };   // White
    const ColorRgba coreColor  { 255, 85, 255, 255 };    // Magenta

    renderer.fillRect(sx - 45, sy - 130, 90, 130, armorColor);
    renderer.fillRect(sx - 40, sy - 125, 80, 120, darkColor);

    renderer.fillRect(sx - 35, sy - 120, 70, 16, armorColor);
    renderer.fillRect(sx - 38, sy - 100, 76, 12, armorColor);
    renderer.fillRect(sx - 32, sy - 40, 64, 18, armorColor);

    const bool flashEyes { mechEyes_.flashTimer > 0.0 };
    const ColorRgba eyeDrawColor { flashEyes ? highlight : (mechEyes_.isExposed ? highlight : coreColor) };
    renderer.fillRect(sx - 26, sy - 84, 18, 10, darkColor);
    renderer.fillRect(sx + 8,  sy - 84, 18, 10, darkColor);
    renderer.fillRect(sx - 24, sy - 82, 14, 6, eyeDrawColor);
    renderer.fillRect(sx + 10, sy - 82, 14, 6, eyeDrawColor);

    if (mechAttack_ == MechHeadAttack::EyeLaserTelegraph) {
        for (int x { sx - 26 }; x >= 0; x -= 12) {
            renderer.fillRect(x, sy - 82, 6, 2, highlight);
        }
    }

    const bool flashMouth { mechMouth_.flashTimer > 0.0 };
    if (mechMouth_.isExposed) {
        const ColorRgba mouthColor { flashMouth ? highlight : coreColor };
        renderer.fillRect(sx - 24, sy - 34, 48, 22, darkColor);
        renderer.fillRect(sx - 20, sy - 30, 40, 14, mouthColor);
    } else {
        renderer.fillRect(sx - 24, sy - 34, 48, 22, armorColor);
        renderer.drawLine(sx - 24, sy - 23, sx + 24, sy - 23, darkColor);
        renderer.drawLine(sx - 12, sy - 34, sx - 12, sy - 12, darkColor);
        renderer.drawLine(sx + 12, sy - 34, sx + 12, sy - 12, darkColor);
    }
}

void BossController::renderCruiser(CgaPaletteRenderer& renderer, const Camera2D& camera) const noexcept
{
    const Vec2 screenPos { camera.worldToScreen(cruiserPos_) };
    const int sx { static_cast<int>(std::round(screenPos.x)) };
    const int sy { static_cast<int>(std::round(screenPos.y)) };

    const ColorRgba hullColor  { 85, 255, 255, 255 };    // Cyan
    const ColorRgba darkColor  { 0, 0, 0, 255 };         // Black
    const ColorRgba highlight  { 255, 255, 255, 255 };   // White
    const ColorRgba coreColor  { 255, 85, 255, 255 };    // Magenta

    renderer.fillRect(sx - 70, sy - 18, 140, 36, hullColor);
    renderer.fillRect(sx - 65, sy - 14, 130, 28, darkColor);
    renderer.fillRect(sx - 55, sy - 10, 110, 20, hullColor);

    if (cruiserPort_.health > 0.0) {
        const ColorRgba podColor { cruiserPort_.flashTimer > 0.0 ? highlight : coreColor };
        renderer.fillRect(sx - 60, sy + 6, 26, 14, podColor);
        renderer.fillRect(sx - 52, sy + 18, 6, 8, highlight);
    } else {
        renderer.fillRect(sx - 60, sy + 6, 26, 6, darkColor);
    }

    if (cruiserStarboard_.health > 0.0) {
        const ColorRgba podColor { cruiserStarboard_.flashTimer > 0.0 ? highlight : coreColor };
        renderer.fillRect(sx + 34, sy + 6, 26, 14, podColor);
        renderer.fillRect(sx + 46, sy + 18, 6, 8, highlight);
    } else {
        renderer.fillRect(sx + 34, sy + 6, 26, 6, darkColor);
    }

    if (isShieldActive()) {
        renderer.drawCircle(sx, sy, 22, highlight);
        renderer.drawCircle(sx, sy, 20, hullColor);
    } else {
        const ColorRgba coreDrawColor { cruiserCore_.flashTimer > 0.0 ? highlight : coreColor };
        renderer.fillCircle(sx, sy, 14, coreDrawColor);
        renderer.drawCircle(sx, sy, 16, highlight);
    }
}

void BossController::renderBombsAndBeams(CgaPaletteRenderer& renderer, const Camera2D& camera) const noexcept
{
    const ColorRgba beamColor { 255, 255, 255, 255 };
    const ColorRgba bombColor { 255, 85, 255, 255 };

    for (const auto& beam : eyeLasers_) {
        if (!beam.active) {
            continue;
        }
        const Vec2 p0 { camera.worldToScreen(Vec2{ beam.targetX, beam.elevationY }) };
        const Vec2 p1 { camera.worldToScreen(Vec2{ beam.originX, beam.elevationY }) };
        const int y { static_cast<int>(std::round(p0.y)) };
        const int x0 { std::max(0, static_cast<int>(std::round(p0.x))) };
        const int x1 { std::min(CgaPaletteRenderer::kNativeWidth, static_cast<int>(std::round(p1.x))) };
        renderer.fillRect(x0, y - 2, x1 - x0, 5, beamColor);
    }

    for (const auto& bomb : clusterBombs_) {
        if (!bomb.active) {
            continue;
        }
        const Vec2 scr { camera.worldToScreen(bomb.position) };
        const int bx { static_cast<int>(std::round(scr.x)) };
        const int by { static_cast<int>(std::round(scr.y)) };
        renderer.fillCircle(bx, by, 4, bombColor);
        renderer.drawCircle(bx, by, 5, beamColor);
    }

    for (const auto& flak : flakShells_) {
        if (!flak.active) {
            continue;
        }
        const Vec2 scr { camera.worldToScreen(flak.position) };
        const int fx { static_cast<int>(std::round(scr.x)) };
        const int fy { static_cast<int>(std::round(scr.y)) };
        renderer.fillRect(fx - 2, fy - 2, 4, 4, beamColor);
    }
}

void BossController::renderBossHud(CgaPaletteRenderer& renderer) const noexcept
{
    constexpr int kBarWidth { 140 };
    constexpr int kBarHeight { 8 };
    constexpr int kBarX { (CgaPaletteRenderer::kNativeWidth - kBarWidth) / 2 };
    constexpr int kBarY { 8 };

    const ColorRgba frameColor { 255, 255, 255, 255 };
    const ColorRgba fillColor  { 255, 85, 255, 255 };
    const ColorRgba darkColor   { 0, 0, 0, 255 };

    renderer.drawRect(kBarX - 1, kBarY - 1, kBarWidth + 2, kBarHeight + 2, frameColor);
    renderer.fillRect(kBarX, kBarY, kBarWidth, kBarHeight, darkColor);

    const double frac { getHealthFraction() };
    const int fillW { static_cast<int>(std::round(frac * static_cast<double>(kBarWidth))) };
    if (fillW > 0) {
        for (int px { 0 }; px < fillW; px += 4) {
            const int segW { std::min(3, fillW - px) };
            renderer.fillRect(kBarX + px, kBarY + 1, segW, kBarHeight - 2, fillColor);
        }
    }
}

BossPhase BossController::getActivePhase() const noexcept
{
    return currentPhase_;
}

bool BossController::isBossActive() const noexcept
{
    return currentPhase_ != BossPhase::Inactive && currentPhase_ != BossPhase::VictoryCleared;
}

bool BossController::isBossDefeated() const noexcept
{
    return currentPhase_ == BossPhase::VictoryCleared;
}

double BossController::getHealthFraction() const noexcept
{
    if (currentPhase_ >= BossPhase::MechHeadEntrance && currentPhase_ <= BossPhase::MechHeadDefeated) {
        const double total { config_.mechHeadMaxHealth };
        const double current { mechMouth_.health + mechEyes_.health };
        return std::clamp(current / total, 0.0, 1.0);
    }
    if (currentPhase_ >= BossPhase::CruiserDescent && currentPhase_ <= BossPhase::CruiserDefeated) {
        const double total { (config_.cruiserTurretHealth * 2.0) + config_.cruiserCoreHealth };
        const double current { cruiserPort_.health + cruiserStarboard_.health + cruiserCore_.health };
        return std::clamp(current / total, 0.0, 1.0);
    }
    return 0.0;
}

const BossControllerConfig& BossController::getConfig() const noexcept
{
    return config_;
}

const Vec2& BossController::getMechHeadPos() const noexcept
{
    return mechHeadPos_;
}

const Vec2& BossController::getCruiserPos() const noexcept
{
    return cruiserPos_;
}

bool BossController::isShieldActive() const noexcept
{
    return (cruiserPort_.health > 0.0 || cruiserStarboard_.health > 0.0);
}

double BossController::getRemainingBossHp() const noexcept
{
    if (currentPhase_ >= BossPhase::MechHeadEntrance && currentPhase_ <= BossPhase::MechHeadDefeated) {
        return mechMouth_.health + mechEyes_.health;
    }
    if (currentPhase_ >= BossPhase::CruiserDescent && currentPhase_ <= BossPhase::CruiserDefeated) {
        return cruiserPort_.health + cruiserStarboard_.health + cruiserCore_.health;
    }
    return 0.0;
}

} // namespace montsoft_offroad
