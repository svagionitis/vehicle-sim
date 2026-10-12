#include "ObstacleManager.h"

#include <algorithm>
#include <cmath>

#include "apps/EarthPatrol/CourseManager.h"
#include "libs/VehicleSimCore/dynamics/EarthRoverDynamics.h"
#include "libs/VehicleSimCore/physics/RigidBody2D.h"

namespace earth_patrol {

ObstacleManager::ObstacleManager() noexcept
{
}

void ObstacleManager::generateObstacles(const CourseManager& course, int biomeIndex)
{
    obstacles_.clear();

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
            // White barrier with orange stripes
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
}

void ObstacleManager::reset() noexcept
{
    for (auto& obs : obstacles_) {
        obs.cleared = false;
    }
}

const std::vector<Obstacle>& ObstacleManager::getObstacles() const noexcept
{
    return obstacles_;
}

} // namespace earth_patrol
