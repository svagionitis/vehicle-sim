#include "apps/MontsoftOffroad/EnemyManager.h"

#include <algorithm>
#include <cmath>

namespace montsoft_offroad {

using vehicle_sim::math::Vec2;
using vehicle_sim::physics::RigidBody2D;
using vehicle_sim::terrain::Heightfield1D;

namespace {

constexpr double kGravity { 9.81 };

} // namespace

EnemyManager::EnemyManager() noexcept
    : EnemyManager(EnemyManagerConfig{})
{
}

EnemyManager::EnemyManager(const EnemyManagerConfig& config) noexcept
    : config_{ config }
{
    enemies_.reserve(config_.maxEnemies);
    projectiles_.reserve(config_.maxProjectiles);

    // Initialize Archetype Configurations
    // 0: ScoutDirtBike
    archetypeConfigs_[static_cast<std::size_t>(EnemyType::ScoutDirtBike)] = EnemyConfig{
        25.0,                         // maxHealth
        20.0,                         // baseSpeed
        1.0,                          // collisionRadius
        Vec2{ 1.8, 1.0 },             // dimensions
        999.0,                        // attackCooldown (no ranged attack)
        150U                          // scoreValue
    };

    // 1: RedChaseBuggy
    archetypeConfigs_[static_cast<std::size_t>(EnemyType::RedChaseBuggy)] = EnemyConfig{
        80.0,                         // maxHealth
        22.0,                         // baseSpeed
        1.3,                          // collisionRadius
        Vec2{ 2.4, 1.2 },             // dimensions
        999.0,                        // attackCooldown (ramming only)
        300U                          // scoreValue
    };

    // 2: BuriedLandmine
    archetypeConfigs_[static_cast<std::size_t>(EnemyType::BuriedLandmine)] = EnemyConfig{
        10.0,                         // maxHealth
        0.0,                          // baseSpeed
        0.7,                          // collisionRadius
        Vec2{ 1.0, 0.4 },             // dimensions
        999.0,                        // attackCooldown (proximity detonator)
        50U                           // scoreValue
    };

    // 3: ArtilleryCrawler
    archetypeConfigs_[static_cast<std::size_t>(EnemyType::ArtilleryCrawler)] = EnemyConfig{
        120.0,                        // maxHealth
        6.0,                          // baseSpeed
        1.5,                          // collisionRadius
        Vec2{ 2.8, 1.4 },             // dimensions
        2.6,                          // attackCooldown
        450U                          // scoreValue
    };

    // 4: StrafingJet
    archetypeConfigs_[static_cast<std::size_t>(EnemyType::StrafingJet)] = EnemyConfig{
        40.0,                         // maxHealth
        40.0,                         // baseSpeed
        1.4,                          // collisionRadius
        Vec2{ 2.6, 1.0 },             // dimensions
        0.35,                         // attackCooldown
        250U                          // scoreValue
    };

    // 5: HoveringSaucer
    archetypeConfigs_[static_cast<std::size_t>(EnemyType::HoveringSaucer)] = EnemyConfig{
        60.0,                         // maxHealth
        16.0,                         // baseSpeed
        1.2,                          // collisionRadius
        Vec2{ 2.2, 1.0 },             // dimensions
        1.8,                          // attackCooldown
        350U                          // scoreValue
    };

    // 6: BouncingDrone
    archetypeConfigs_[static_cast<std::size_t>(EnemyType::BouncingDrone)] = EnemyConfig{
        35.0,                         // maxHealth
        14.0,                         // baseSpeed
        0.9,                          // collisionRadius
        Vec2{ 1.2, 1.2 },             // dimensions
        999.0,                        // attackCooldown (contact explosive)
        200U                          // scoreValue
    };
}

bool EnemyManager::spawnEnemy(
    EnemyType type,
    const Vec2& position,
    const Vec2& initialVelocity
) noexcept
{
    const auto typeIdx { static_cast<std::size_t>(type) };
    const auto& archetype { archetypeConfigs_[typeIdx] };

    // Find existing inactive slot
    for (auto& enemy : enemies_) {
        if (!enemy.active) {
            enemy.id = nextEntityId_++;
            enemy.type = type;
            enemy.position = position;
            enemy.velocity = initialVelocity;
            enemy.orientation = 0.0;
            enemy.health = archetype.maxHealth;
            enemy.maxHealth = archetype.maxHealth;
            enemy.attackTimer = 0.0;
            enemy.flashTimer = 0.0;
            enemy.animTimer = 0.0;
            enemy.sinePhase = 0.0;
            enemy.active = true;
            return true;
        }
    }

    // Allocate up to maxEnemies
    if (enemies_.size() < config_.maxEnemies) {
        EnemyEntity newEnemy {};
        newEnemy.id = nextEntityId_++;
        newEnemy.type = type;
        newEnemy.position = position;
        newEnemy.velocity = initialVelocity;
        newEnemy.orientation = 0.0;
        newEnemy.health = archetype.maxHealth;
        newEnemy.maxHealth = archetype.maxHealth;
        newEnemy.attackTimer = 0.0;
        newEnemy.flashTimer = 0.0;
        newEnemy.animTimer = 0.0;
        newEnemy.sinePhase = 0.0;
        newEnemy.active = true;
        enemies_.push_back(newEnemy);
        return true;
    }

    return false;
}

void EnemyManager::spawnFormation(
    std::size_t stageIndex,
    std::size_t milestoneIndex,
    double playerX,
    const Heightfield1D* terrain
) noexcept
{
    if (lastSpawnedStage_ == stageIndex && lastSpawnedMilestone_ == milestoneIndex) {
        return;
    }
    lastSpawnedStage_ = stageIndex;
    lastSpawnedMilestone_ = milestoneIndex;

    const auto getGround { [terrain](double x) noexcept -> double {
        return (terrain != nullptr) ? terrain->getHeight(x) : 0.0;
    }};

    switch (stageIndex) {
    case 0: // Stage 1: Sun-Baked Dunes (Scout Dirt Bikes & Landmines)
        if (milestoneIndex == 0) {
            const double sx { playerX + config_.spawnLeadDistance };
            spawnEnemy(EnemyType::ScoutDirtBike, Vec2{ sx, getGround(sx) + 0.6 }, Vec2{ -10.0, 0.0 });
        } else if (milestoneIndex == 1) {
            const double sx { playerX + config_.spawnLeadDistance };
            spawnEnemy(EnemyType::BuriedLandmine, Vec2{ sx, getGround(sx) }, Vec2{ 0.0, 0.0 });
        } else if (milestoneIndex == 2) {
            const double sx1 { playerX + config_.spawnLeadDistance };
            const double sx2 { sx1 + 8.0 };
            spawnEnemy(EnemyType::ScoutDirtBike, Vec2{ sx1, getGround(sx1) + 0.6 }, Vec2{ -12.0, 0.0 });
            spawnEnemy(EnemyType::ScoutDirtBike, Vec2{ sx2, getGround(sx2) + 0.6 }, Vec2{ -12.0, 0.0 });
        } else if (milestoneIndex == 3) {
            const double sx { playerX + 28.0 };
            spawnEnemy(EnemyType::BuriedLandmine, Vec2{ sx, getGround(sx) }, Vec2{ 0.0, 0.0 });
            spawnEnemy(EnemyType::BuriedLandmine, Vec2{ sx + 10.0, getGround(sx + 10.0) }, Vec2{ 0.0, 0.0 });
        } else if (milestoneIndex == 4) {
            const double sx { playerX + config_.spawnLeadDistance };
            spawnEnemy(EnemyType::ScoutDirtBike, Vec2{ sx, getGround(sx) + 0.6 }, Vec2{ -14.0, 0.0 });
            spawnEnemy(EnemyType::ScoutDirtBike, Vec2{ sx + 6.0, getGround(sx + 6.0) + 0.6 }, Vec2{ -14.0, 0.0 });
            spawnEnemy(EnemyType::ScoutDirtBike, Vec2{ sx + 12.0, getGround(sx + 12.0) + 0.6 }, Vec2{ -14.0, 0.0 });
        }
        break;

    case 1: // Stage 2: Red Rock Arroyos (Scout Bikes + Red Chase Buggies)
        if (milestoneIndex == 0 || milestoneIndex == 2) {
            const double rx { playerX - config_.spawnRearDistance };
            spawnEnemy(EnemyType::RedChaseBuggy, Vec2{ rx, getGround(rx) + 0.7 }, Vec2{ 15.0, 0.0 });
        } else if (milestoneIndex == 1 || milestoneIndex == 3) {
            const double sx { playerX + config_.spawnLeadDistance };
            spawnEnemy(EnemyType::ScoutDirtBike, Vec2{ sx, getGround(sx) + 0.6 }, Vec2{ -12.0, 0.0 });
            spawnEnemy(EnemyType::BuriedLandmine, Vec2{ sx + 6.0, getGround(sx + 6.0) }, Vec2{ 0.0, 0.0 });
        } else if (milestoneIndex == 4) {
            const double rx { playerX - config_.spawnRearDistance };
            const double sx { playerX + config_.spawnLeadDistance };
            spawnEnemy(EnemyType::RedChaseBuggy, Vec2{ rx, getGround(rx) + 0.7 }, Vec2{ 18.0, 0.0 });
            spawnEnemy(EnemyType::ScoutDirtBike, Vec2{ sx, getGround(sx) + 0.6 }, Vec2{ -14.0, 0.0 });
        }
        break;

    case 2: // Stage 3: Alkali Flats (Strafing Jet Fighters + Landmines)
        if (milestoneIndex % 2 == 0) {
            const double jx { playerX + 32.0 };
            spawnEnemy(EnemyType::StrafingJet, Vec2{ jx, getGround(jx) + 12.0 }, Vec2{ -35.0, -8.0 });
        } else {
            const double sx { playerX + 24.0 };
            spawnEnemy(EnemyType::BuriedLandmine, Vec2{ sx, getGround(sx) }, Vec2{ 0.0, 0.0 });
            spawnEnemy(EnemyType::ScoutDirtBike, Vec2{ sx + 8.0, getGround(sx + 8.0) + 0.6 }, Vec2{ -12.0, 0.0 });
        }
        break;

    case 3: // Stage 4: Petrified Ridge (Bouncing Drone Squares + Artillery Crawlers)
        if (milestoneIndex == 0 || milestoneIndex == 3) {
            const double cx { playerX + 32.0 };
            spawnEnemy(EnemyType::ArtilleryCrawler, Vec2{ cx, getGround(cx) + 0.8 }, Vec2{ -4.0, 0.0 });
        } else {
            const double dx { playerX + 26.0 };
            spawnEnemy(EnemyType::BouncingDrone, Vec2{ dx, getGround(dx) + 5.0 }, Vec2{ -12.0, 0.0 });
        }
        break;

    case 4: // Stage 5: Volcanic Badlands (Artillery Crawlers + Hovering Saucers)
        if (milestoneIndex % 2 == 0) {
            const double sx { playerX + 28.0 };
            spawnEnemy(EnemyType::HoveringSaucer, Vec2{ sx, getGround(sx) + 8.5 }, Vec2{ -8.0, 0.0 });
        } else {
            const double cx { playerX + 34.0 };
            spawnEnemy(EnemyType::ArtilleryCrawler, Vec2{ cx, getGround(cx) + 0.8 }, Vec2{ -4.0, 0.0 });
        }
        break;

    case 5: // Stage 6: Scrap Canyon (Strafing Jets + Red Buggy Trios)
        {
            const double rx { playerX - config_.spawnRearDistance };
            const double jx { playerX + 34.0 };
            spawnEnemy(EnemyType::RedChaseBuggy, Vec2{ rx, getGround(rx) + 0.7 }, Vec2{ 16.0, 0.0 });
            spawnEnemy(EnemyType::StrafingJet, Vec2{ jx, getGround(jx) + 11.0 }, Vec2{ -38.0, -7.0 });
        }
        break;

    case 6: // Stage 7: Alpine Scree (Jets + Saucers + Drones)
        {
            const double jx { playerX + 32.0 };
            const double sx { playerX + 26.0 };
            spawnEnemy(EnemyType::StrafingJet, Vec2{ jx, getGround(jx) + 12.0 }, Vec2{ -35.0, -8.0 });
            spawnEnemy(EnemyType::HoveringSaucer, Vec2{ sx, getGround(sx) + 8.0 }, Vec2{ -6.0, 0.0 });
        }
        break;

    case 7: // Stage 8: Secret Fortress (Crawlers + Saucers + Landmines Gauntlet)
    default:
        {
            const double cx { playerX + 30.0 };
            const double mx { playerX + 18.0 };
            spawnEnemy(EnemyType::ArtilleryCrawler, Vec2{ cx, getGround(cx) + 0.8 }, Vec2{ -4.0, 0.0 });
            spawnEnemy(EnemyType::HoveringSaucer, Vec2{ cx + 8.0, getGround(cx + 8.0) + 8.5 }, Vec2{ -8.0, 0.0 });
            spawnEnemy(EnemyType::BuriedLandmine, Vec2{ mx, getGround(mx) }, Vec2{ 0.0, 0.0 });
        }
        break;
    }
}

void EnemyManager::updateGroundAi(
    EnemyEntity& enemy,
    double dt,
    const Heightfield1D* terrain,
    const Vec2& playerPos
) noexcept
{
    const double groundY { (terrain != nullptr) ? terrain->getHeight(enemy.position.x) : 0.0 };
    const double slope { (terrain != nullptr) ? terrain->getSlope(enemy.position.x) : 0.0 };
    const double targetTheta { std::atan(slope) };

    // Smooth heading angle relaxation
    enemy.orientation += (1.0 - std::exp(-14.0 * dt)) * (targetTheta - enemy.orientation);

    switch (enemy.type) {
    case EnemyType::ScoutDirtBike:
        enemy.position.x += enemy.velocity.x * dt;
        enemy.position.y = groundY + 0.55;
        enemy.animTimer += dt * std::abs(enemy.velocity.x) * 4.0;
        break;

    case EnemyType::RedChaseBuggy:
        {
            // PD Speed Matching to player position from behind
            const double targetSpeed { 20.0 };
            const double speedErr { (playerPos.x > enemy.position.x) ? (targetSpeed - enemy.velocity.x) : (-targetSpeed - enemy.velocity.x) };
            const double accel { std::clamp(speedErr * 4.5, -12.0, 16.0) };
            enemy.velocity.x += accel * dt;
            enemy.position.x += enemy.velocity.x * dt;
            enemy.position.y = groundY + 0.65;
            enemy.animTimer += dt * std::abs(enemy.velocity.x) * 3.5;
        }
        break;

    case EnemyType::BuriedLandmine:
        enemy.position.y = groundY;
        enemy.animTimer += dt;
        break;

    case EnemyType::ArtilleryCrawler:
        enemy.position.x += enemy.velocity.x * dt;
        enemy.position.y = groundY + 0.75;
        enemy.animTimer += dt * std::abs(enemy.velocity.x) * 2.0;

        // Firing Ballistic Mortar at Player
        enemy.attackTimer += dt;
        if (enemy.attackTimer >= archetypeConfigs_[static_cast<std::size_t>(EnemyType::ArtilleryCrawler)].attackCooldown) {
            enemy.attackTimer = 0.0;
            const double dx { playerPos.x - enemy.position.x };
            // Fire shell if player within reasonable distance
            if (std::abs(dx) > 6.0 && std::abs(dx) < 40.0) {
                constexpr double tFlight { 1.8 };
                const double vxShell { dx / tFlight };
                const double vyShell { (playerPos.y - enemy.position.y + 0.5 * kGravity * tFlight * tFlight) / tFlight };
                fireEnemyBomb(enemy.position + Vec2{ 0.0, 0.8 }, Vec2{ vxShell, vyShell }, 35.0, 2.5, true);
            }
        }
        break;

    default:
        break;
    }
}

void EnemyManager::updateAerialAi(
    EnemyEntity& enemy,
    double dt,
    const Vec2& playerPos
) noexcept
{
    switch (enemy.type) {
    case EnemyType::StrafingJet:
        enemy.position += enemy.velocity * dt;
        enemy.animTimer += dt;

        // Flatten dive once below 6m
        if (enemy.position.y < 5.5 && enemy.velocity.y < 0.0) {
            enemy.velocity.y = 0.0;
        }

        // Fire twin machine gun tracers when over player
        enemy.attackTimer += dt;
        if (enemy.attackTimer >= archetypeConfigs_[static_cast<std::size_t>(EnemyType::StrafingJet)].attackCooldown) {
            const double dx { std::abs(enemy.position.x - playerPos.x) };
            if (dx < 12.0) {
                enemy.attackTimer = 0.0;
                fireEnemyBullet(enemy.position + Vec2{ -0.6, -0.3 }, Vec2{ enemy.velocity.x * 0.4, -22.0 }, 8.0);
                fireEnemyBullet(enemy.position + Vec2{ 0.6, -0.3 }, Vec2{ enemy.velocity.x * 0.4, -22.0 }, 8.0);
            }
        }
        break;

    case EnemyType::HoveringSaucer:
        {
            // Track player horizontally with lead
            const double targetX { playerPos.x + 3.0 };
            const double dx { targetX - enemy.position.x };
            enemy.velocity.x = std::clamp(dx * 2.5, -16.0, 16.0);
            enemy.position.x += enemy.velocity.x * dt;
            enemy.animTimer += dt * 8.0;

            // Drop pulsed plasma bombs
            enemy.attackTimer += dt;
            if (enemy.attackTimer >= archetypeConfigs_[static_cast<std::size_t>(EnemyType::HoveringSaucer)].attackCooldown) {
                if (std::abs(playerPos.x - enemy.position.x) < 8.0) {
                    enemy.attackTimer = 0.0;
                    fireEnemyBomb(enemy.position - Vec2{ 0.0, 0.5 }, Vec2{ 0.0, -16.0 }, 25.0, 2.0, false);
                }
            }
        }
        break;

    case EnemyType::BouncingDrone:
        {
            enemy.position.x += enemy.velocity.x * dt;
            enemy.sinePhase += dt * 3.6;
            constexpr double kAmplitude { 2.8 };
            enemy.position.y += kAmplitude * std::cos(enemy.sinePhase) * 3.6 * dt;
            enemy.animTimer += dt * 5.0;
        }
        break;

    default:
        break;
    }
}

void EnemyManager::fireEnemyBullet(
    const Vec2& origin,
    const Vec2& velocity,
    double damage
) noexcept
{
    for (auto& proj : projectiles_) {
        if (!proj.active) {
            proj.position = origin;
            proj.velocity = velocity;
            proj.damage = damage;
            proj.blastRadius = 1.0;
            proj.age = 0.0;
            proj.maxLifespan = 2.5;
            proj.isBallistic = false;
            proj.active = true;
            return;
        }
    }

    if (projectiles_.size() < config_.maxProjectiles) {
        EnemyProjectile proj {};
        proj.position = origin;
        proj.velocity = velocity;
        proj.damage = damage;
        proj.blastRadius = 1.0;
        proj.age = 0.0;
        proj.maxLifespan = 2.5;
        proj.isBallistic = false;
        proj.active = true;
        projectiles_.push_back(proj);
    }
}

void EnemyManager::fireEnemyBomb(
    const Vec2& origin,
    const Vec2& velocity,
    double damage,
    double radius,
    bool isBallistic
) noexcept
{
    for (auto& proj : projectiles_) {
        if (!proj.active) {
            proj.position = origin;
            proj.velocity = velocity;
            proj.damage = damage;
            proj.blastRadius = radius;
            proj.age = 0.0;
            proj.maxLifespan = 3.5;
            proj.isBallistic = isBallistic;
            proj.active = true;
            return;
        }
    }

    if (projectiles_.size() < config_.maxProjectiles) {
        EnemyProjectile proj {};
        proj.position = origin;
        proj.velocity = velocity;
        proj.damage = damage;
        proj.blastRadius = radius;
        proj.age = 0.0;
        proj.maxLifespan = 3.5;
        proj.isBallistic = isBallistic;
        proj.active = true;
        projectiles_.push_back(proj);
    }
}

void EnemyManager::updateProjectiles(
    double dt,
    const Heightfield1D* terrain,
    RigidBody2D& truckBody,
    ParticleSystem& particleSystem
) noexcept
{
    const Vec2 truckPos { truckBody.getPosition() };
    constexpr double kTruckHitRadius { 1.6 };

    for (auto& proj : projectiles_) {
        if (!proj.active) {
            continue;
        }

        proj.age += dt;
        if (proj.age >= proj.maxLifespan) {
            proj.active = false;
            continue;
        }

        if (proj.isBallistic) {
            proj.velocity.y -= kGravity * dt;
        }
        proj.position += proj.velocity * dt;

        // Ground collision check
        if (terrain != nullptr) {
            const double groundY { terrain->getHeight(proj.position.x) };
            if (proj.position.y <= groundY) {
                proj.active = false;
                particleSystem.emitExplosionShrapnel(Vec2{ proj.position.x, groundY }, proj.blastRadius, 0.8, 0U);
                continue;
            }
        }

        // Player truck hit check
        const double distToTruck { (proj.position - truckPos).length() };
        if (distToTruck <= kTruckHitRadius) {
            proj.active = false;
            particleSystem.emitExplosionShrapnel(proj.position, proj.blastRadius, 1.0, 0U);
            playerHealth_ -= proj.damage;
            truckBody.applyLinearImpulse(proj.velocity.normalized() * 320.0);
            if (playerHealth_ <= 0.0) {
                truckDestroyed_ = true;
            }
        }
    }
}

void EnemyManager::resolveTruckContacts(
    RigidBody2D& truckBody,
    ParticleSystem& particleSystem
) noexcept
{
    const Vec2 truckPos { truckBody.getPosition() };

    for (auto& enemy : enemies_) {
        if (!enemy.active) {
            continue;
        }

        const double dist { (enemy.position - truckPos).length() };

        // 1. Buried Landmine Proximity Detonation
        if (enemy.type == EnemyType::BuriedLandmine && dist < 1.3) {
            enemy.active = false;
            particleSystem.emitExplosionShrapnel(enemy.position, 3.2, 1.2, 0U);
            truckBody.applyLinearImpulse(Vec2{ 0.0, 4500.0 }); // Launch truck upward!
            playerHealth_ -= 35.0;
            if (playerHealth_ <= 0.0) {
                truckDestroyed_ = true;
            }
            continue;
        }

        // 2. Red Chase Buggy Ramming Contact
        if (enemy.type == EnemyType::RedChaseBuggy && dist < 2.0) {
            // If truck is airborne and above the buggy, player successfully jumped over it
            if (truckPos.y > enemy.position.y + 0.85) {
                accumulatedScore_ += 100U;
                particleSystem.emitThrusterSparks(enemy.position + Vec2{ 0.0, 0.5 }, Vec2{ 0.0, 5.0 });
                continue;
            }
            // Apply forward impulse bump to truck
            const Vec2 pushDir { (truckPos.x > enemy.position.x) ? Vec2{ 1.0, 0.1 } : Vec2{ -1.0, 0.1 } };
            truckBody.applyLinearImpulse(pushDir * 600.0);
            enemy.velocity.x *= 0.5; // Buggy loses momentum
            playerHealth_ -= 10.0;
            if (playerHealth_ <= 0.0) {
                truckDestroyed_ = true;
            }
            continue;
        }

        // 3. Scout Dirt Bike Contact
        if (enemy.type == EnemyType::ScoutDirtBike && dist < 1.5) {
            if (truckPos.y > enemy.position.y + 0.70) {
                accumulatedScore_ += 100U;
                continue;
            }
            enemy.active = false;
            particleSystem.emitExplosionShrapnel(enemy.position, 2.0, 0.8, 0U);
            playerHealth_ -= 10.0;
            if (playerHealth_ <= 0.0) {
                truckDestroyed_ = true;
            }
            continue;
        }

        // 4. Drone Square Mid-air Contact Crash
        if (enemy.type == EnemyType::BouncingDrone && dist < 1.4) {
            enemy.active = false;
            particleSystem.emitExplosionShrapnel(enemy.position, 2.5, 1.0, 0U);
            truckBody.applyLinearImpulse(Vec2{ -400.0, -200.0 });
            playerHealth_ -= 25.0;
            if (playerHealth_ <= 0.0) {
                truckDestroyed_ = true;
            }
            continue;
        }
    }
}

void EnemyManager::update(
    double dt,
    const Heightfield1D* terrain,
    const Camera2D& camera,
    RigidBody2D& truckBody,
    ParticleSystem& particleSystem
) noexcept
{
    const Vec2 playerPos { truckBody.getPosition() };
    const double camX { camera.getPosition().x };

    for (auto& enemy : enemies_) {
        if (!enemy.active) {
            continue;
        }

        // Update hit flash timer
        if (enemy.flashTimer > 0.0) {
            enemy.flashTimer = std::max(0.0, enemy.flashTimer - dt);
        }

        // Kinematics and AI behaviors
        if (enemy.type == EnemyType::ScoutDirtBike ||
            enemy.type == EnemyType::RedChaseBuggy ||
            enemy.type == EnemyType::BuriedLandmine ||
            enemy.type == EnemyType::ArtilleryCrawler) {
            updateGroundAi(enemy, dt, terrain, playerPos);
        } else {
            updateAerialAi(enemy, dt, playerPos);
        }

        // Despawn if far behind camera
        if (enemy.position.x < camX - config_.despawnDistance) {
            enemy.active = false;
        }
    }

    // Hostile projectiles and contacts
    updateProjectiles(dt, terrain, truckBody, particleSystem);
    resolveTruckContacts(truckBody, particleSystem);
}

void EnemyManager::syncCombatTargets(std::vector<CombatTargetAabb>& targets) const noexcept
{
    targets.clear();
    for (const auto& enemy : enemies_) {
        if (!enemy.active) {
            continue;
        }

        const auto typeIdx { static_cast<std::size_t>(enemy.type) };
        const auto& dim { archetypeConfigs_[typeIdx].dimensions };

        CombatTargetAabb target {};
        target.targetId = enemy.id;
        target.type = (enemy.type == EnemyType::ScoutDirtBike || enemy.type == EnemyType::RedChaseBuggy)
            ? CombatTargetType::GroundScout
            : (enemy.type == EnemyType::ArtilleryCrawler)
            ? CombatTargetType::GroundHeavy
            : (enemy.type == EnemyType::StrafingJet)
            ? CombatTargetType::AerialFighter
            : CombatTargetType::AerialSaucer;

        target.minBounds = enemy.position - dim * 0.5;
        target.maxBounds = enemy.position + dim * 0.5;
        target.health = enemy.health;
        target.isAlive = true;

        targets.push_back(target);
    }
}

void EnemyManager::applyCombatDamage(
    const std::vector<CombatTargetAabb>& targets,
    ParticleSystem& particleSystem,
    std::size_t stageIndex
) noexcept
{
    for (const auto& target : targets) {
        for (auto& enemy : enemies_) {
            if (enemy.active && enemy.id == target.targetId) {
                if (target.health < enemy.health) {
                    enemy.flashTimer = 0.12; // White hit flash
                    enemy.health = target.health;
                }

                if (!target.isAlive || enemy.health <= 0.0) {
                    enemy.active = false;
                    const auto typeIdx { static_cast<std::size_t>(enemy.type) };
                    accumulatedScore_ += archetypeConfigs_[typeIdx].scoreValue;
                    particleSystem.emitExplosionShrapnel(enemy.position, 2.8, 1.2, stageIndex);
                }
                break;
            }
        }
    }
}

void EnemyManager::reset() noexcept
{
    for (auto& enemy : enemies_) {
        enemy.active = false;
    }
    for (auto& proj : projectiles_) {
        proj.active = false;
    }
    accumulatedScore_ = 0U;
    truckDestroyed_ = false;
    playerHealth_ = 100.0;
    lastSpawnedMilestone_ = 999U;
    lastSpawnedStage_ = 999U;
}

std::size_t EnemyManager::getActiveCount() const noexcept
{
    std::size_t count { 0U };
    for (const auto& enemy : enemies_) {
        if (enemy.active) {
            ++count;
        }
    }
    return count;
}

const std::vector<EnemyEntity>& EnemyManager::getEnemies() const noexcept
{
    return enemies_;
}

const std::vector<EnemyProjectile>& EnemyManager::getProjectiles() const noexcept
{
    return projectiles_;
}

std::uint32_t EnemyManager::getAccumulatedScore() const noexcept
{
    return accumulatedScore_;
}

bool EnemyManager::checkTruckDestroyed() const noexcept
{
    return truckDestroyed_;
}

const EnemyManagerConfig& EnemyManager::getConfig() const noexcept
{
    return config_;
}

// -----------------------------------------------------------------------------
// CGA Retro Rasterization Routines
// -----------------------------------------------------------------------------

void EnemyManager::renderDirtBike(
    CgaPaletteRenderer& renderer,
    const Camera2D& camera,
    const EnemyEntity& enemy
) const noexcept
{
    const Vec2 screenPos { camera.worldToScreen(enemy.position) };
    const int sx { static_cast<int>(std::round(screenPos.x)) };
    const int sy { static_cast<int>(std::round(screenPos.y)) };

    const bool flash { enemy.flashTimer > 0.0 };
    const ColorRgba mainCol { flash ? ColorRgba{ 255, 255, 255, 255 } : ColorRgba{ 85, 255, 255, 255 } }; // Cyan
    const ColorRgba tireCol { flash ? ColorRgba{ 255, 255, 255, 255 } : ColorRgba{ 255, 85, 255, 255 } }; // Magenta
    const ColorRgba riderCol { ColorRgba{ 255, 255, 255, 255 } };

    // Front & rear spinning wheels
    renderer.fillRect(sx - 8, sy - 3, 4, 4, tireCol);
    renderer.fillRect(sx + 4, sy - 3, 4, 4, tireCol);

    // Frame chassis bar
    renderer.drawLine(sx - 6, sy - 2, sx + 5, sy - 2, mainCol);
    renderer.drawLine(sx, sy - 2, sx + 2, sy - 7, mainCol);

    // Rider silhouette
    renderer.fillRect(sx - 2, sy - 8, 3, 4, riderCol);
    renderer.fillRect(sx - 3, sy - 11, 4, 3, mainCol); // Helmet
}

void EnemyManager::renderChaseBuggy(
    CgaPaletteRenderer& renderer,
    const Camera2D& camera,
    const EnemyEntity& enemy
) const noexcept
{
    const Vec2 screenPos { camera.worldToScreen(enemy.position) };
    const int sx { static_cast<int>(std::round(screenPos.x)) };
    const int sy { static_cast<int>(std::round(screenPos.y)) };

    const bool flash { enemy.flashTimer > 0.0 };
    const ColorRgba bodyCol { flash ? ColorRgba{ 255, 255, 255, 255 } : ColorRgba{ 255, 85, 85, 255 } };   // Red / Magenta
    const ColorRgba tireCol { flash ? ColorRgba{ 255, 255, 255, 255 } : ColorRgba{ 85, 255, 255, 255 } };  // Cyan
    const ColorRgba cageCol { ColorRgba{ 255, 255, 255, 255 } };

    // Oversized rear knobby tires & front tires
    renderer.fillRect(sx - 11, sy - 4, 6, 6, tireCol);
    renderer.fillRect(sx + 5, sy - 3, 5, 5, tireCol);

    // Lower chassis wedge
    renderer.fillRect(sx - 9, sy - 4, 16, 4, bodyCol);

    // Roll-cage struts
    renderer.drawLine(sx - 6, sy - 4, sx - 2, sy - 9, cageCol);
    renderer.drawLine(sx - 2, sy - 9, sx + 4, sy - 4, cageCol);
    renderer.drawLine(sx - 6, sy - 9, sx + 2, sy - 9, cageCol);

    // Dual spotlights
    renderer.setPixel(sx + 2, sy - 10, ColorRgba{ 255, 255, 85, 255 });
}

void EnemyManager::renderLandmine(
    CgaPaletteRenderer& renderer,
    const Camera2D& camera,
    const EnemyEntity& enemy
) const noexcept
{
    const Vec2 screenPos { camera.worldToScreen(enemy.position) };
    const int sx { static_cast<int>(std::round(screenPos.x)) };
    const int sy { static_cast<int>(std::round(screenPos.y)) };

    const bool blink { std::fmod(enemy.animTimer, 0.25) < 0.125 };
    const ColorRgba ledCol { blink ? ColorRgba{ 255, 255, 255, 255 } : ColorRgba{ 255, 85, 85, 255 } };

    // Beveled ground disc
    renderer.fillRect(sx - 4, sy - 2, 8, 3, ColorRgba{ 85, 255, 255, 255 });
    // Blinking trigger LED
    renderer.setPixel(sx - 1, sy - 3, ledCol);
    renderer.setPixel(sx, sy - 3, ledCol);
}

void EnemyManager::renderArtillery(
    CgaPaletteRenderer& renderer,
    const Camera2D& camera,
    const EnemyEntity& enemy
) const noexcept
{
    const Vec2 screenPos { camera.worldToScreen(enemy.position) };
    const int sx { static_cast<int>(std::round(screenPos.x)) };
    const int sy { static_cast<int>(std::round(screenPos.y)) };

    const bool flash { enemy.flashTimer > 0.0 };
    const ColorRgba hullCol { flash ? ColorRgba{ 255, 255, 255, 255 } : ColorRgba{ 255, 85, 255, 255 } };
    const ColorRgba treadCol { ColorRgba{ 85, 255, 255, 255 } };
    const ColorRgba barrelCol { ColorRgba{ 255, 255, 255, 255 } };

    // Caterpillar tread base
    renderer.fillRect(sx - 12, sy - 3, 24, 4, treadCol);
    // Armored hull
    renderer.fillRect(sx - 10, sy - 7, 20, 5, hullCol);
    // Turret dome
    renderer.fillRect(sx - 5, sy - 10, 10, 4, hullCol);
    // Elevating mortar cannon barrel pointing up-left
    renderer.drawLine(sx, sy - 9, sx - 8, sy - 15, barrelCol);
}

void EnemyManager::renderStrafingJet(
    CgaPaletteRenderer& renderer,
    const Camera2D& camera,
    const EnemyEntity& enemy
) const noexcept
{
    const Vec2 screenPos { camera.worldToScreen(enemy.position) };
    const int sx { static_cast<int>(std::round(screenPos.x)) };
    const int sy { static_cast<int>(std::round(screenPos.y)) };

    const bool flash { enemy.flashTimer > 0.0 };
    const ColorRgba jetCol { flash ? ColorRgba{ 255, 255, 255, 255 } : ColorRgba{ 85, 255, 255, 255 } };
    const ColorRgba flameCol { ColorRgba{ 255, 85, 85, 255 } };

    // Needle delta fuselage
    renderer.drawLine(sx + 10, sy, sx - 10, sy, jetCol);
    renderer.fillRect(sx - 4, sy - 2, 8, 4, jetCol);

    // Tail fin & delta wings
    renderer.drawLine(sx + 6, sy - 1, sx + 8, sy - 5, jetCol);
    renderer.drawLine(sx - 2, sy - 3, sx - 7, sy - 6, jetCol);
    renderer.drawLine(sx - 2, sy + 3, sx - 7, sy + 6, jetCol);

    // Jet thruster flames behind (right side moving left)
    renderer.fillRect(sx + 11, sy - 1, 3, 2, flameCol);
}

void EnemyManager::renderSaucer(
    CgaPaletteRenderer& renderer,
    const Camera2D& camera,
    const EnemyEntity& enemy
) const noexcept
{
    const Vec2 screenPos { camera.worldToScreen(enemy.position) };
    const int sx { static_cast<int>(std::round(screenPos.x)) };
    const int sy { static_cast<int>(std::round(screenPos.y)) };

    const bool flash { enemy.flashTimer > 0.0 };
    const ColorRgba rimCol { flash ? ColorRgba{ 255, 255, 255, 255 } : ColorRgba{ 255, 85, 255, 255 } };
    const ColorRgba domeCol { ColorRgba{ 85, 255, 255, 255 } };

    // Elliptical disc saucer rim
    renderer.fillRect(sx - 9, sy - 1, 18, 3, rimCol);
    renderer.fillRect(sx - 7, sy - 2, 14, 5, rimCol);

    // Upper glass dome
    renderer.fillRect(sx - 3, sy - 5, 6, 3, domeCol);

    // Pulsing perimeter lights
    const int lightPhase { static_cast<int>(std::floor(enemy.animTimer)) % 3 };
    renderer.setPixel(sx - 6 + lightPhase * 2, sy, ColorRgba{ 255, 255, 255, 255 });
    renderer.setPixel(sx + 4 - lightPhase * 2, sy, ColorRgba{ 255, 255, 255, 255 });
}

void EnemyManager::renderDroneSquare(
    CgaPaletteRenderer& renderer,
    const Camera2D& camera,
    const EnemyEntity& enemy
) const noexcept
{
    const Vec2 screenPos { camera.worldToScreen(enemy.position) };
    const int sx { static_cast<int>(std::round(screenPos.x)) };
    const int sy { static_cast<int>(std::round(screenPos.y)) };

    const bool flash { enemy.flashTimer > 0.0 };
    const ColorRgba frameCol { flash ? ColorRgba{ 255, 255, 255, 255 } : ColorRgba{ 85, 255, 255, 255 } };
    const ColorRgba coreCol { ColorRgba{ 255, 85, 255, 255 } };

    // Outer armored diamond
    renderer.drawRect(sx - 5, sy - 5, 10, 10, frameCol);
    // Rotating gyro core
    renderer.fillRect(sx - 2, sy - 2, 4, 4, coreCol);

    // Micro thruster sparks
    renderer.setPixel(sx - 6, sy, ColorRgba{ 255, 255, 255, 255 });
    renderer.setPixel(sx + 5, sy, ColorRgba{ 255, 255, 255, 255 });
}

void EnemyManager::render(
    CgaPaletteRenderer& renderer,
    const Camera2D& camera
) const noexcept
{
    // 1. Render all active enemies
    for (const auto& enemy : enemies_) {
        if (!enemy.active) {
            continue;
        }

        switch (enemy.type) {
        case EnemyType::ScoutDirtBike:
            renderDirtBike(renderer, camera, enemy);
            break;
        case EnemyType::RedChaseBuggy:
            renderChaseBuggy(renderer, camera, enemy);
            break;
        case EnemyType::BuriedLandmine:
            renderLandmine(renderer, camera, enemy);
            break;
        case EnemyType::ArtilleryCrawler:
            renderArtillery(renderer, camera, enemy);
            break;
        case EnemyType::StrafingJet:
            renderStrafingJet(renderer, camera, enemy);
            break;
        case EnemyType::HoveringSaucer:
            renderSaucer(renderer, camera, enemy);
            break;
        case EnemyType::BouncingDrone:
            renderDroneSquare(renderer, camera, enemy);
            break;
        }
    }

    // 2. Render all active hostile projectiles
    for (const auto& proj : projectiles_) {
        if (!proj.active) {
            continue;
        }

        const Vec2 screenPos { camera.worldToScreen(proj.position) };
        const int px { static_cast<int>(std::round(screenPos.x)) };
        const int py { static_cast<int>(std::round(screenPos.y)) };

        if (px >= 0 && px < CgaPaletteRenderer::kNativeWidth &&
            py >= 0 && py < CgaPaletteRenderer::kNativeHeight) {
            if (proj.isBallistic) {
                // Artillery mortar shell: 3x3 diamond
                renderer.fillRect(px - 1, py - 1, 3, 3, ColorRgba{ 255, 85, 85, 255 });
                renderer.setPixel(px, py, ColorRgba{ 255, 255, 255, 255 });
            } else if (proj.damage > 15.0) {
                // Saucer plasma bomb: 4x4 pulsing orb
                renderer.fillRect(px - 2, py - 2, 4, 4, ColorRgba{ 255, 85, 255, 255 });
                renderer.fillRect(px - 1, py - 1, 2, 2, ColorRgba{ 255, 255, 255, 255 });
            } else {
                // Jet tracer bullet: 2x3 vertical streak
                renderer.fillRect(px - 1, py - 1, 2, 3, ColorRgba{ 255, 255, 85, 255 });
            }
        }
    }
}

} // namespace montsoft_offroad
