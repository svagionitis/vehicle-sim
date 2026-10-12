#include "ObstacleManager.h"

#include <algorithm>
#include <cmath>

#include "apps/EarthPatrol/CourseManager.h"
#include "libs/VehicleSimCore/dynamics/EarthRoverDynamics.h"
#include "libs/VehicleSimCore/physics/RigidBody2D.h"
#include "libs/VehicleSimCore/terrain/Heightfield1D.h"

namespace earth_patrol {

ObstacleManager::ObstacleManager() noexcept
{
}

void ObstacleManager::generateObstacles(const CourseManager& course, int biomeIndex)
{
    obstacles_.clear();
    dynamicObstacles_.clear();

    const auto& heightfield { course.getHeightfield() };

    struct ObstacleSpawn {
        double x { 0.0 };
        ObstacleType type { ObstacleType::SmallRock };
    };

    std::vector<ObstacleSpawn> spawns { ObstacleSpawn { 28.0, ObstacleType::SmallRock },
        ObstacleSpawn { 65.0, ObstacleType::SmallRock },
        ObstacleSpawn { 110.0, (biomeIndex == 2) ? ObstacleType::FallenLog : ObstacleType::ConcreteBarrier },
        ObstacleSpawn { 160.0, ObstacleType::LargeBoulder }, ObstacleSpawn { 210.0, ObstacleType::SmallRock },
        ObstacleSpawn { 260.0, ObstacleType::LargeBoulder }, ObstacleSpawn { 310.0, ObstacleType::SmallRock },
        ObstacleSpawn { 355.0, (biomeIndex == 2) ? ObstacleType::FallenLog : ObstacleType::ConcreteBarrier },
        ObstacleSpawn { 405.0, ObstacleType::SmallRock }, ObstacleSpawn { 455.0, ObstacleType::LargeBoulder },
        ObstacleSpawn { 488.0, (biomeIndex == 2) ? ObstacleType::FallenLog : ObstacleType::ConcreteBarrier } };

    for (const auto& sp : spawns) {
        const double elev { heightfield.getHeight(sp.x) };

        Obstacle obs { };
        obs.type = sp.type;
        obs.worldPos = vehicle_sim::math::Vec2 { sp.x, elev };
        obs.cleared = false;

        switch (sp.type) {
        case ObstacleType::SmallRock:
            obs.width = 0.65;
            obs.height = 0.45;
            obs.fatal = false;
            break;
        case ObstacleType::LargeBoulder:
            obs.width = 1.10;
            obs.height = 0.95;
            obs.fatal = true;
            break;
        case ObstacleType::FallenLog:
            obs.width = 1.60;
            obs.height = 0.55;
            obs.fatal = false;
            break;
        case ObstacleType::ConcreteBarrier:
            obs.width = 0.85;
            obs.height = 1.05;
            obs.fatal = true;
            break;
        }

        obstacles_.push_back(obs);
    }

    // Biome-specific dynamic moving hazards
    if (biomeIndex == 0) {
        // Highway: Autonomous Roadwork Drones patrolling back and forth
        DynamicObstacle drone1 { };
        drone1.type = DynamicObstacleType::PatrolDrone;
        drone1.radius = 0.45;
        drone1.position = vehicle_sim::math::Vec2 { 130.0, heightfield.getHeight(130.0) + drone1.radius };
        drone1.velocity = vehicle_sim::math::Vec2 { 3.2, 0.0 };
        drone1.patrolMinX = 118.0;
        drone1.patrolMaxX = 144.0;
        drone1.fatal = true;
        dynamicObstacles_.push_back(drone1);

        DynamicObstacle drone2 { };
        drone2.type = DynamicObstacleType::PatrolDrone;
        drone2.radius = 0.45;
        drone2.position = vehicle_sim::math::Vec2 { 330.0, heightfield.getHeight(330.0) + drone2.radius };
        drone2.velocity = vehicle_sim::math::Vec2 { -3.0, 0.0 };
        drone2.patrolMinX = 315.0;
        drone2.patrolMaxX = 345.0;
        drone2.fatal = true;
        dynamicObstacles_.push_back(drone2);

        DynamicObstacle drone3 { };
        drone3.type = DynamicObstacleType::PatrolDrone;
        drone3.radius = 0.45;
        drone3.position = vehicle_sim::math::Vec2 { 470.0, heightfield.getHeight(470.0) + drone3.radius };
        drone3.velocity = vehicle_sim::math::Vec2 { 3.5, 0.0 };
        drone3.patrolMinX = 455.0;
        drone3.patrolMaxX = 482.0;
        drone3.fatal = true;
        dynamicObstacles_.push_back(drone3);
    } else if (biomeIndex == 1) {
        // Desert: Rolling Boulders and Windblown Debris
        DynamicObstacle boulder1 { };
        boulder1.type = DynamicObstacleType::RollingBoulder;
        boulder1.radius = 0.55;
        boulder1.position = vehicle_sim::math::Vec2 { 180.0, heightfield.getHeight(180.0) + boulder1.radius };
        boulder1.velocity = vehicle_sim::math::Vec2 { -4.5, 0.0 };
        boulder1.fatal = true;
        dynamicObstacles_.push_back(boulder1);

        DynamicObstacle boulder2 { };
        boulder2.type = DynamicObstacleType::RollingBoulder;
        boulder2.radius = 0.60;
        boulder2.position = vehicle_sim::math::Vec2 { 380.0, heightfield.getHeight(380.0) + boulder2.radius };
        boulder2.velocity = vehicle_sim::math::Vec2 { -5.0, 0.0 };
        boulder2.fatal = true;
        dynamicObstacles_.push_back(boulder2);

        DynamicObstacle debris1 { };
        debris1.type = DynamicObstacleType::TumblingDebris;
        debris1.radius = 0.35;
        debris1.position = vehicle_sim::math::Vec2 { 85.0, heightfield.getHeight(85.0) + debris1.radius };
        debris1.velocity = vehicle_sim::math::Vec2 { -3.0, 2.0 };
        debris1.fatal = false;
        dynamicObstacles_.push_back(debris1);

        DynamicObstacle debris2 { };
        debris2.type = DynamicObstacleType::TumblingDebris;
        debris2.radius = 0.35;
        debris2.position = vehicle_sim::math::Vec2 { 280.0, heightfield.getHeight(280.0) + debris2.radius };
        debris2.velocity = vehicle_sim::math::Vec2 { -3.5, 2.0 };
        debris2.fatal = false;
        dynamicObstacles_.push_back(debris2);
    } else {
        // Alpine: Heavy downhill Rolling Boulders
        DynamicObstacle boulder1 { };
        boulder1.type = DynamicObstacleType::RollingBoulder;
        boulder1.radius = 0.50;
        boulder1.position = vehicle_sim::math::Vec2 { 135.0, heightfield.getHeight(135.0) + boulder1.radius };
        boulder1.velocity = vehicle_sim::math::Vec2 { -5.0, 0.0 };
        boulder1.fatal = true;
        dynamicObstacles_.push_back(boulder1);

        DynamicObstacle boulder2 { };
        boulder2.type = DynamicObstacleType::RollingBoulder;
        boulder2.radius = 0.65;
        boulder2.position = vehicle_sim::math::Vec2 { 235.0, heightfield.getHeight(235.0) + boulder2.radius };
        boulder2.velocity = vehicle_sim::math::Vec2 { -6.0, 0.0 };
        boulder2.fatal = true;
        dynamicObstacles_.push_back(boulder2);

        DynamicObstacle boulder3 { };
        boulder3.type = DynamicObstacleType::RollingBoulder;
        boulder3.radius = 0.55;
        boulder3.position = vehicle_sim::math::Vec2 { 425.0, heightfield.getHeight(425.0) + boulder3.radius };
        boulder3.velocity = vehicle_sim::math::Vec2 { -5.5, 0.0 };
        boulder3.fatal = true;
        dynamicObstacles_.push_back(boulder3);
    }
}

void ObstacleManager::updateDynamic(double dt, const vehicle_sim::terrain::Heightfield1D& terrain) noexcept
{
    for (auto& dyn : dynamicObstacles_) {
        dyn.position.x += dyn.velocity.x * dt;

        if (dyn.type == DynamicObstacleType::RollingBoulder) {
            const double groundY { terrain.getHeight(dyn.position.x) };
            dyn.position.y = groundY + dyn.radius;
            const double omega { -dyn.velocity.x / dyn.radius };
            dyn.rotationAngle += omega * dt;
        } else if (dyn.type == DynamicObstacleType::PatrolDrone) {
            if (dyn.position.x <= dyn.patrolMinX && dyn.velocity.x < 0.0) {
                dyn.velocity.x = std::abs(dyn.velocity.x);
            } else if (dyn.position.x >= dyn.patrolMaxX && dyn.velocity.x > 0.0) {
                dyn.velocity.x = -std::abs(dyn.velocity.x);
            }
            const double groundY { terrain.getHeight(dyn.position.x) };
            dyn.position.y = groundY + dyn.radius;
            dyn.rotationAngle += 6.0 * dt;
        } else if (dyn.type == DynamicObstacleType::TumblingDebris) {
            dyn.velocity.y -= 9.81 * dt;
            dyn.position.y += dyn.velocity.y * dt;
            const double groundY { terrain.getHeight(dyn.position.x) + dyn.radius };
            if (dyn.position.y <= groundY) {
                dyn.position.y = groundY;
                dyn.velocity.y = std::abs(dyn.velocity.y) * 0.70;
                if (dyn.velocity.y < 0.6) {
                    dyn.velocity.y = 2.4;
                }
            }
            dyn.rotationAngle += 5.0 * dt;
        }
    }
}

bool ObstacleManager::checkDynamicColl(const vehicle_sim::physics::RigidBody2D& body, bool& outFatal) noexcept
{
    outFatal = false;
    const auto& pos { body.getPosition() };

    constexpr double halfWidth { 1.10 };
    constexpr double halfHeight { 0.35 };
    const double chassisMinX { pos.x - halfWidth };
    const double chassisMaxX { pos.x + halfWidth };
    const double chassisMinY { pos.y - halfHeight };
    const double chassisMaxY { pos.y + halfHeight };

    for (auto& dyn : dynamicObstacles_) {
        const double dynMinX { dyn.position.x - dyn.radius };
        const double dynMaxX { dyn.position.x + dyn.radius };
        const double dynMinY { dyn.position.y - dyn.radius };
        const double dynMaxY { dyn.position.y + dyn.radius };

        const bool overlapX { chassisMaxX >= dynMinX && chassisMinX <= dynMaxX };
        const bool overlapY { chassisMaxY >= dynMinY && chassisMinY <= dynMaxY };

        if (overlapX && overlapY) {
            outFatal = dyn.fatal;
            return true;
        }

        if (pos.x > dynMaxX && !dyn.cleared) {
            dyn.cleared = true;
        }
    }

    return false;
}

bool ObstacleManager::checkCollision(const vehicle_sim::physics::RigidBody2D& body,
    const vehicle_sim::dynamics::EarthRoverDynamics& rover, bool& outFatal) noexcept
{
    static_cast<void>(rover);
    outFatal = false;

    const auto& pos { body.getPosition() };

    // Fatal pit plunge test: fallen below safe terrain threshold
    if (pos.y < -1.8) {
        outFatal = true;
        return true;
    }

    // Chassis 2D AABB bounding box
    constexpr double halfWidth { 1.10 };
    constexpr double halfHeight { 0.35 };
    const double chassisMinX { pos.x - halfWidth };
    const double chassisMaxX { pos.x + halfWidth };
    const double chassisMinY { pos.y - halfHeight };
    const double chassisMaxY { pos.y + halfHeight };

    for (auto& obs : obstacles_) {
        const double obsMinX { obs.worldPos.x - (obs.width * 0.5) };
        const double obsMaxX { obs.worldPos.x + (obs.width * 0.5) };
        const double obsMinY { obs.worldPos.y };
        const double obsMaxY { obs.worldPos.y + obs.height };

        const bool overlapX { chassisMaxX >= obsMinX && chassisMinX <= obsMaxX };
        const bool overlapY { chassisMaxY >= obsMinY && chassisMinY <= obsMaxY };

        if (overlapX && overlapY) {
            if (obs.fatal) {
                outFatal = true;
                return true;
            }
            // Non-fatal contact
            outFatal = false;
            return true;
        }

        if (pos.x > obsMaxX && !obs.cleared) {
            obs.cleared = true;
        }
    }

    // Check dynamic moving obstacles
    if (checkDynamicColl(body, outFatal)) {
        return true;
    }

    return false;
}

void ObstacleManager::render(SDL_Renderer* renderer, const Camera2D& camera) const noexcept
{
    if (renderer == nullptr) {
        return;
    }

    const double minX { camera.getVisibleMinX() - 2.0 };
    const double maxX { camera.getVisibleMaxX() + 2.0 };
    const double ppm { camera.getConfig().pixelsPerMeter };

    // 1. Render static obstacles
    for (const auto& obs : obstacles_) {
        if (obs.worldPos.x < minX || obs.worldPos.x > maxX) {
            continue;
        }

        const auto screenBase { camera.worldToScreen(obs.worldPos) };
        const int wPx { static_cast<int>(obs.width * ppm) };
        const int hPx { static_cast<int>(obs.height * ppm) };
        const int xPx { static_cast<int>(screenBase.x) - (wPx / 2) };
        const int yPx { static_cast<int>(screenBase.y) - hPx };

        switch (obs.type) {
        case ObstacleType::SmallRock: {
            SDL_SetRenderDrawColor(renderer, 120, 115, 110, 255);
            SDL_Rect rockRect { xPx, yPx, wPx, hPx };
            SDL_RenderFillRect(renderer, &rockRect);
            SDL_SetRenderDrawColor(renderer, 150, 145, 140, 255);
            SDL_Rect rockHighlight { xPx + 2, yPx + 2, wPx - 4, hPx / 2 };
            SDL_RenderFillRect(renderer, &rockHighlight);
            break;
        }
        case ObstacleType::LargeBoulder: {
            SDL_SetRenderDrawColor(renderer, 85, 78, 75, 255);
            SDL_Rect boulderRect { xPx, yPx, wPx, hPx };
            SDL_RenderFillRect(renderer, &boulderRect);
            SDL_SetRenderDrawColor(renderer, 120, 110, 105, 255);
            SDL_Rect boulderCrest { xPx + 4, yPx + 4, wPx - 8, hPx / 3 };
            SDL_RenderFillRect(renderer, &boulderCrest);
            break;
        }
        case ObstacleType::FallenLog: {
            SDL_SetRenderDrawColor(renderer, 105, 65, 38, 255);
            SDL_Rect logRect { xPx, yPx, wPx, hPx };
            SDL_RenderFillRect(renderer, &logRect);
            SDL_SetRenderDrawColor(renderer, 140, 95, 55, 255);
            SDL_Rect logBark { xPx + 3, yPx + 3, wPx - 6, hPx / 2 };
            SDL_RenderFillRect(renderer, &logBark);
            break;
        }
        case ObstacleType::ConcreteBarrier: {
            SDL_SetRenderDrawColor(renderer, 220, 220, 225, 255);
            SDL_Rect barrierRect { xPx, yPx, wPx, hPx };
            SDL_RenderFillRect(renderer, &barrierRect);
            SDL_SetRenderDrawColor(renderer, 240, 110, 30, 255);
            SDL_Rect stripeA { xPx, yPx + (hPx / 4), wPx, hPx / 4 };
            SDL_Rect stripeB { xPx, yPx + (hPx * 3 / 4), wPx, hPx / 4 };
            SDL_RenderFillRect(renderer, &stripeA);
            SDL_RenderFillRect(renderer, &stripeB);
            break;
        }
        }
    }

    // 2. Render dynamic moving obstacles
    for (const auto& dyn : dynamicObstacles_) {
        if (dyn.position.x < minX || dyn.position.x > maxX) {
            continue;
        }

        const auto screenCenter { camera.worldToScreen(dyn.position) };
        const int cx { static_cast<int>(screenCenter.x) };
        const int cy { static_cast<int>(screenCenter.y) };
        const int rPx { static_cast<int>(dyn.radius * ppm) };

        switch (dyn.type) {
        case DynamicObstacleType::RollingBoulder: {
            // Boulder body
            SDL_SetRenderDrawColor(renderer, 92, 85, 80, 255);
            SDL_Rect boulderRect { cx - rPx, cy - rPx, rPx * 2, rPx * 2 };
            SDL_RenderFillRect(renderer, &boulderRect);
            SDL_SetRenderDrawColor(renderer, 130, 122, 115, 255);
            SDL_Rect crestRect { cx - rPx + 3, cy - rPx + 3, (rPx * 2) - 6, rPx };
            SDL_RenderFillRect(renderer, &crestRect);

            // Rotating radial spokes illustrating angular rolling motion
            SDL_SetRenderDrawColor(renderer, 55, 50, 48, 255);
            for (int k { 0 }; k < 4; ++k) {
                const double angle { dyn.rotationAngle + (static_cast<double>(k) * 1.57079632679) };
                const int sx { cx + static_cast<int>(std::cos(angle) * (rPx - 2)) };
                const int sy { cy - static_cast<int>(std::sin(angle) * (rPx - 2)) };
                SDL_RenderDrawLine(renderer, cx, cy, sx, sy);
            }
            break;
        }
        case DynamicObstacleType::PatrolDrone: {
            const int wPx { static_cast<int>(dyn.radius * 2.2 * ppm) };
            const int hPx { static_cast<int>(dyn.radius * 1.4 * ppm) };
            const int xPx { cx - (wPx / 2) };
            const int yPx { cy - (hPx / 2) };

            // Yellow industrial body
            SDL_SetRenderDrawColor(renderer, 240, 190, 30, 255);
            SDL_Rect bodyRect { xPx, yPx, wPx, hPx };
            SDL_RenderFillRect(renderer, &bodyRect);

            // Black hazard warning stripe
            SDL_SetRenderDrawColor(renderer, 35, 35, 35, 255);
            SDL_Rect stripeRect { xPx + (wPx / 3), yPx, wPx / 3, hPx };
            SDL_RenderFillRect(renderer, &stripeRect);

            // Flashing emergency beacon on roof
            const bool beaconActive { std::sin(dyn.rotationAngle * 2.0) > 0.0 };
            if (beaconActive) {
                SDL_SetRenderDrawColor(renderer, 255, 60, 40, 255);
            } else {
                SDL_SetRenderDrawColor(renderer, 140, 20, 10, 255);
            }
            SDL_Rect beaconRect { cx - 3, yPx - 6, 6, 6 };
            SDL_RenderFillRect(renderer, &beaconRect);
            break;
        }
        case DynamicObstacleType::TumblingDebris: {
            // Rotating shards
            SDL_SetRenderDrawColor(renderer, 175, 140, 90, 255);
            for (int k { 0 }; k < 3; ++k) {
                const double angle { dyn.rotationAngle + (static_cast<double>(k) * 2.09439510239) };
                const int px { cx + static_cast<int>(std::cos(angle) * (rPx * 0.75)) };
                const int py { cy - static_cast<int>(std::sin(angle) * (rPx * 0.75)) };
                SDL_Rect shardRect { px - 3, py - 3, 6, 6 };
                SDL_RenderFillRect(renderer, &shardRect);
            }
            break;
        }
        }
    }
}

void ObstacleManager::reset() noexcept
{
    for (auto& obs : obstacles_) {
        obs.cleared = false;
    }
    for (auto& dyn : dynamicObstacles_) {
        dyn.cleared = false;
    }
}

int ObstacleManager::getClearedCount() const noexcept
{
    int count { 0 };
    for (const auto& obs : obstacles_) {
        if (obs.cleared) {
            count++;
        }
    }
    for (const auto& dyn : dynamicObstacles_) {
        if (dyn.cleared) {
            count++;
        }
    }
    return count;
}

const std::vector<Obstacle>& ObstacleManager::getObstacles() const noexcept
{
    return obstacles_;
}

const std::vector<DynamicObstacle>& ObstacleManager::getDynamicObstacles() const noexcept
{
    return dynamicObstacles_;
}

} // namespace earth_patrol
