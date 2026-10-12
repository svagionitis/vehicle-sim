#include "CourseManager.h"

#include <algorithm>
#include <cmath>

namespace earth_patrol {

namespace {
    inline constexpr double kPi { 3.14159265358979323846 };
} // namespace

CourseManager::CourseManager()
    : CourseManager(CourseConfig { })
{
}

CourseManager::CourseManager(const CourseConfig& config)
    : config_ { config }
{
    buildCourse(0);
}

void CourseManager::buildCourse(int biomeIndex)
{
    heightfield_.clearSamples();
    heightfield_.setBaseHeight(0.0);
    heightfield_.setInterpolation(vehicle_sim::terrain::HeightfieldInterp::CubicHermite);
    heightfield_.setBoundaryMode(vehicle_sim::terrain::HeightfieldBoundary::Clamp);

    std::vector<vehicle_sim::terrain::HeightSample1D> samples { };

    const double endX { config_.totalLength + 60.0 };
    constexpr double dx { 2.5 };

    // Define craters: {startX, width, depth}
    struct CraterDef {
        double startX { 0.0 };
        double width { 0.0 };
        double depth { 0.0 };
    };

    std::vector<CraterDef> craters {
        CraterDef { 45.0, 3.5, 1.2 }, // Small pothole before Point A
        CraterDef { 80.0, 4.0, 1.5 }, // Pothole near Point A
        CraterDef { 140.0, 5.5, 2.4 }, // Medium crater before Point B
        CraterDef { 185.0, 4.5, 1.8 }, // Crater before Point B
        CraterDef { 235.0, 7.0, 3.5 }, // Wide ravine before Point C
        CraterDef { 280.0, 5.0, 2.0 }, // Pothole before Point C
        CraterDef { 330.0, 6.0, 2.8 }, // Deep sinkhole before Point D
        CraterDef { 380.0, 5.5, 2.2 }, // Trench before Point D
        CraterDef { 430.0, 7.5, 3.8 }, // Grand chasm before Point E
        CraterDef { 475.0, 4.5, 1.8 } // Final pothole before finish
    };

    double hillFrequency { 0.04 };
    double hillAmplitude { 0.8 };
    if (biomeIndex == 1) { // Desert Badlands
        hillFrequency = 0.06;
        hillAmplitude = 1.3;
    } else if (biomeIndex == 2) { // Alpine Forest Pass
        hillFrequency = 0.03;
        hillAmplitude = 1.8;
    }

    for (double x { -40.0 }; x <= endX; x += dx) {
        double y { 0.0 };

        // Rolling Earth mounds
        if (x > 15.0 && x < config_.totalLength) {
            y += std::sin(x * hillFrequency) * hillAmplitude;
            y += std::sin(x * (hillFrequency * 2.3) + 0.5) * (hillAmplitude * 0.35);
        }

        // Apply crater depressions
        for (const auto& crater : craters) {
            if (x >= crater.startX && x <= (crater.startX + crater.width)) {
                const double progress { (x - crater.startX) / crater.width };
                // Parabolic / sinusoidal crater depression profile
                const double depthFactor { std::sin(progress * kPi) };
                y -= crater.depth * depthFactor;
            }
        }

        samples.push_back(vehicle_sim::terrain::HeightSample1D { x, y });
    }

    heightfield_.setSamples(samples);

    // Setup checkpoints A through E
    checkpoints_.clear();
    const char letters[] { 'A', 'B', 'C', 'D', 'E' };
    for (std::size_t i { 0 }; i < 5; ++i) {
        const double cpX { static_cast<double>(i + 1) * config_.segmentLength };
        checkpoints_.push_back(CheckpointInfo { letters[i], cpX, false });
    }

    reset();
}

void CourseManager::update(double roverWorldX, double dt) noexcept
{
    roverX_ = roverWorldX;

    if (!stageComplete_ && remainingTime_ > 0.0) {
        remainingTime_ = std::max(0.0, remainingTime_ - dt);
    }

    for (std::size_t i { 0 }; i < checkpoints_.size(); ++i) {
        auto& cp { checkpoints_[i] };
        if (!cp.reached && roverX_ >= cp.worldX) {
            cp.reached = true;
            score_ += config_.checkpointBonus;

            if (cp.letter == 'E') {
                stageComplete_ = true;
                score_ += config_.stageClearBonus;
                score_ += static_cast<int>(remainingTime_ * 100.0);
            }
        }
    }

    // Determine upcoming checkpoint
    nextCheckpointIdx_ = checkpoints_.size() - 1;
    for (std::size_t i { 0 }; i < checkpoints_.size(); ++i) {
        if (!checkpoints_[i].reached) {
            nextCheckpointIdx_ = i;
            break;
        }
    }
}

void CourseManager::reset() noexcept
{
    roverX_ = 0.0;
    remainingTime_ = config_.parTime;
    score_ = 0;
    stageComplete_ = false;
    nextCheckpointIdx_ = 0;

    for (auto& cp : checkpoints_) {
        cp.reached = false;
    }
}

const vehicle_sim::terrain::Heightfield1D& CourseManager::getHeightfield() const noexcept
{
    return heightfield_;
}

vehicle_sim::terrain::Heightfield1D& CourseManager::getHeightfield() noexcept
{
    return heightfield_;
}

char CourseManager::getCurrentCheckpoint() const noexcept
{
    if (nextCheckpointIdx_ < checkpoints_.size()) {
        return checkpoints_[nextCheckpointIdx_].letter;
    }
    return 'E';
}

double CourseManager::getDistanceToNext() const noexcept
{
    if (nextCheckpointIdx_ < checkpoints_.size()) {
        return std::max(0.0, checkpoints_[nextCheckpointIdx_].worldX - roverX_);
    }
    return 0.0;
}

double CourseManager::getProgressFraction() const noexcept
{
    if (config_.totalLength <= 0.0) {
        return 0.0;
    }
    return std::clamp(roverX_ / config_.totalLength, 0.0, 1.0);
}

bool CourseManager::isStageComplete() const noexcept
{
    return stageComplete_;
}

double CourseManager::getRemainingTime() const noexcept
{
    return remainingTime_;
}

int CourseManager::getScore() const noexcept
{
    return score_;
}

void CourseManager::addScore(int points) noexcept
{
    score_ += points;
}

const std::vector<CheckpointInfo>& CourseManager::getCheckpoints() const noexcept
{
    return checkpoints_;
}

const CourseConfig& CourseManager::getConfig() const noexcept
{
    return config_;
}

} // namespace earth_patrol
