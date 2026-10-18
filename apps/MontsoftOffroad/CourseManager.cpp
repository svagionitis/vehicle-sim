#include "apps/MontsoftOffroad/CourseManager.h"

#include <algorithm>
#include <cmath>

namespace montsoft_offroad {

using vehicle_sim::terrain::HeightfieldBoundary;
using vehicle_sim::terrain::HeightfieldInterp;
using vehicle_sim::terrain::HeightSample1D;
using vehicle_sim::terrain::MaterialType;
using vehicle_sim::terrain::SurfaceMaterial;

namespace {
constexpr double kPi { 3.14159265358979323846 };
} // namespace

CourseManager::CourseManager()
{
    loadStage(1);
}

void CourseManager::loadStage(int stageIndex)
{
    const int clampedIndex { std::clamp(stageIndex, 1, 8) };

    heightfield_.clearSamples();
    heightfield_.clearMaterialZones();
    heightfield_.setInterpolation(HeightfieldInterp::CubicHermite);
    heightfield_.setBoundaryMode(HeightfieldBoundary::Clamp);

    switch (clampedIndex) {
    case 1:
        generateMojaveDunes();
        break;
    case 2:
        generateRedRockArroyo();
        break;
    case 3:
        generateGreatSaltFlats();
        break;
    case 4:
        generateMuddyQuagmire();
        break;
    case 5:
        generateVolcanicBadlands();
        break;
    case 6:
        generateScrapCanyon();
        break;
    case 7:
        generateAlpineRidge();
        break;
    case 8:
        generateSecretFortress();
        break;
    default:
        generateMojaveDunes();
        break;
    }

    setupCheckpoints(currentStage_.totalLength);
    reset();
}

void CourseManager::setupCheckpoints(double length)
{
    checkpoints_.clear();
    const double step { length / static_cast<double>(kCheckpointsPerStage) };

    const std::array<char, kCheckpointsPerStage> letters { 'A', 'B', 'C', 'D', 'E' };
    const std::array<int, kCheckpointsPerStage> bonuses { 1000, 1500, 2000, 2500, 10000 };

    for (std::size_t i { 0 }; i < kCheckpointsPerStage; ++i) {
        CheckpointMarker marker {};
        marker.letter = letters[i];
        marker.worldX = step * static_cast<double>(i + 1);
        marker.scoreBonus = bonuses[i];
        marker.timeBonus = 15.0;
        marker.reached = false;
        checkpoints_.push_back(marker);
    }
}

void CourseManager::update(double truckX, double dt) noexcept
{
    truckX_ = truckX;

    if (!stageComplete_ && remainingTime_ > 0.0) {
        remainingTime_ = std::max(0.0, remainingTime_ - dt);
    }

    while (nextCheckpointIdx_ < checkpoints_.size()) {
        auto& cp = checkpoints_[nextCheckpointIdx_];
        if (truckX_ >= cp.worldX) {
            cp.reached = true;
            score_ += cp.scoreBonus;
            remainingTime_ += cp.timeBonus;
            ++nextCheckpointIdx_;

            if (nextCheckpointIdx_ >= checkpoints_.size()) {
                stageComplete_ = true;
            }
        } else {
            break;
        }
    }
}

void CourseManager::reset() noexcept
{
    truckX_ = 0.0;
    remainingTime_ = currentStage_.parTime;
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

const StageDescriptor& CourseManager::getCurrentStage() const noexcept
{
    return currentStage_;
}

int CourseManager::getCurrentStageIndex() const noexcept
{
    return currentStage_.stageIndex;
}

const std::vector<CheckpointMarker>& CourseManager::getCheckpoints() const noexcept
{
    return checkpoints_;
}

char CourseManager::getNextCheckpointLetter() const noexcept
{
    if (nextCheckpointIdx_ < checkpoints_.size()) {
        return checkpoints_[nextCheckpointIdx_].letter;
    }
    return 'E';
}

double CourseManager::getDistanceToNext() const noexcept
{
    if (nextCheckpointIdx_ < checkpoints_.size()) {
        return std::max(0.0, checkpoints_[nextCheckpointIdx_].worldX - truckX_);
    }
    return 0.0;
}

double CourseManager::getProgressFraction() const noexcept
{
    if (currentStage_.totalLength <= 0.0) {
        return 0.0;
    }
    return std::clamp(truckX_ / currentStage_.totalLength, 0.0, 1.0);
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

void CourseManager::generateMojaveDunes()
{
    currentStage_.stageIndex = 1;
    currentStage_.title = "Dunes of Mojave";
    currentStage_.biome = "Desert Dunes";
    currentStage_.totalLength = 500.0;
    currentStage_.parTime = 75.0;
    currentStage_.primaryMaterial = SurfaceMaterial::getPreset(MaterialType::Sand);

    heightfield_.setDefaultMaterial(currentStage_.primaryMaterial);

    std::vector<HeightSample1D> samples;
    constexpr double dx { 1.0 };
    for (double x { -20.0 }; x <= 520.0; x += dx) {
        double y { 0.8 * std::sin(2.0 * kPi * x / 40.0) + 0.3 * std::sin(2.0 * kPi * x / 14.0) };

        // Small crater at x in [140, 146]
        if (x >= 140.0 && x <= 146.0) {
            y -= 0.7 * (1.0 - std::cos(2.0 * kPi * (x - 140.0) / 6.0)) * 0.5;
        }
        // Roadside mogul pair at x in [280, 292]
        if (x >= 280.0 && x <= 292.0) {
            y += 0.5 * std::sin(2.0 * kPi * (x - 280.0) / 6.0);
        }

        samples.push_back(HeightSample1D{ x, y });
    }
    heightfield_.setSamples(samples);
}

void CourseManager::generateRedRockArroyo()
{
    currentStage_.stageIndex = 2;
    currentStage_.title = "Red Rock Arroyo";
    currentStage_.biome = "Sandstone Canyon";
    currentStage_.totalLength = 500.0;
    currentStage_.parTime = 80.0;
    currentStage_.primaryMaterial = SurfaceMaterial::getPreset(MaterialType::Dirt);

    heightfield_.setDefaultMaterial(currentStage_.primaryMaterial);

    std::vector<HeightSample1D> samples;
    constexpr double dx { 1.0 };
    for (double x { -20.0 }; x <= 520.0; x += dx) {
        double y { 1.2 * std::sin(2.0 * kPi * x / 60.0) + 0.5 * std::cos(2.0 * kPi * x / 25.0) };

        // Jump chasm 1: x in [165, 185] (ramp at 155-165)
        if (x >= 155.0 && x < 165.0) {
            y += 1.8 * ((x - 155.0) / 10.0);
        } else if (x >= 165.0 && x <= 185.0) {
            y = -3.5;
        }

        // Jump chasm 2: x in [345, 365]
        if (x >= 335.0 && x < 345.0) {
            y += 1.6 * ((x - 335.0) / 10.0);
        } else if (x >= 345.0 && x <= 365.0) {
            y = -4.0;
        }

        samples.push_back(HeightSample1D{ x, y });
    }
    heightfield_.setSamples(samples);
}

void CourseManager::generateGreatSaltFlats()
{
    currentStage_.stageIndex = 3;
    currentStage_.title = "Great Salt Flats";
    currentStage_.biome = "Salt Pan";
    currentStage_.totalLength = 500.0;
    currentStage_.parTime = 65.0;
    currentStage_.primaryMaterial = SurfaceMaterial::getPreset(MaterialType::Tarmac);

    heightfield_.setDefaultMaterial(currentStage_.primaryMaterial);

    std::vector<HeightSample1D> samples;
    constexpr double dx { 1.0 };
    for (double x { -20.0 }; x <= 520.0; x += dx) {
        // High-speed planar surface with fine ripple harmonics
        const double y { 0.12 * std::sin(2.0 * kPi * x / 18.0) + 0.05 * std::sin(2.0 * kPi * x / 7.0) };
        samples.push_back(HeightSample1D{ x, y });
    }
    heightfield_.setSamples(samples);
}

void CourseManager::generateMuddyQuagmire()
{
    currentStage_.stageIndex = 4;
    currentStage_.title = "Muddy Quagmire";
    currentStage_.biome = "Swamp Crossing";
    currentStage_.totalLength = 500.0;
    currentStage_.parTime = 90.0;
    currentStage_.primaryMaterial = SurfaceMaterial::getPreset(MaterialType::Mud);

    heightfield_.setDefaultMaterial(currentStage_.primaryMaterial);

    std::vector<HeightSample1D> samples;
    constexpr double dx { 1.0 };
    for (double x { -20.0 }; x <= 520.0; x += dx) {
        double y { 0.4 * std::sin(2.0 * kPi * x / 30.0) };

        // Mud sinkholes
        if (x >= 90.0 && x <= 110.0) {
            y -= 1.2 * (1.0 - std::cos(2.0 * kPi * (x - 90.0) / 20.0)) * 0.5;
        }
        if (x >= 230.0 && x <= 255.0) {
            y -= 1.4 * (1.0 - std::cos(2.0 * kPi * (x - 230.0) / 25.0)) * 0.5;
        }
        if (x >= 380.0 && x <= 410.0) {
            y -= 1.3 * (1.0 - std::cos(2.0 * kPi * (x - 380.0) / 30.0)) * 0.5;
        }

        samples.push_back(HeightSample1D{ x, y });
    }
    heightfield_.setSamples(samples);
}

void CourseManager::generateVolcanicBadlands()
{
    currentStage_.stageIndex = 5;
    currentStage_.title = "Volcanic Badlands";
    currentStage_.biome = "Lava Crags";
    currentStage_.totalLength = 500.0;
    currentStage_.parTime = 85.0;
    currentStage_.primaryMaterial = SurfaceMaterial::getPreset(MaterialType::Gravel);

    heightfield_.setDefaultMaterial(currentStage_.primaryMaterial);

    std::vector<HeightSample1D> samples;
    constexpr double dx { 1.0 };
    for (double x { -20.0 }; x <= 520.0; x += dx) {
        double y { 1.0 * std::sin(2.0 * kPi * x / 45.0) + 0.6 * std::cos(2.0 * kPi * x / 20.0) };

        // Magma crevasse 1 at [130, 145]
        if (x >= 122.0 && x < 130.0) {
            y += 2.0 * ((x - 122.0) / 8.0);
        } else if (x >= 130.0 && x <= 145.0) {
            y = -5.0;
        }

        // Magma crevasse 2 at [270, 288]
        if (x >= 262.0 && x < 270.0) {
            y += 2.2 * ((x - 262.0) / 8.0);
        } else if (x >= 270.0 && x <= 288.0) {
            y = -5.0;
        }

        samples.push_back(HeightSample1D{ x, y });
    }
    heightfield_.setSamples(samples);
}

void CourseManager::generateScrapCanyon()
{
    currentStage_.stageIndex = 6;
    currentStage_.title = "Scrap Canyon Wasteland";
    currentStage_.biome = "Junkyard";
    currentStage_.totalLength = 500.0;
    currentStage_.parTime = 85.0;
    currentStage_.primaryMaterial = SurfaceMaterial::getPreset(MaterialType::Dirt);

    heightfield_.setDefaultMaterial(currentStage_.primaryMaterial);

    std::vector<HeightSample1D> samples;
    constexpr double dx { 1.0 };
    for (double x { -20.0 }; x <= 520.0; x += dx) {
        // Jagged, stepped debris piles
        double y { 0.5 * std::floor(std::sin(2.0 * kPi * x / 50.0) * 3.0) };
        if (x >= 150.0 && x <= 165.0) {
            y += 1.5;
        }
        if (x >= 300.0 && x <= 320.0) {
            y += 2.0;
        }
        samples.push_back(HeightSample1D{ x, y });
    }
    heightfield_.setSamples(samples);
}

void CourseManager::generateAlpineRidge()
{
    currentStage_.stageIndex = 7;
    currentStage_.title = "Alpine Scree Ridge";
    currentStage_.biome = "Mountain Pass";
    currentStage_.totalLength = 500.0;
    currentStage_.parTime = 90.0;
    currentStage_.primaryMaterial = SurfaceMaterial::getPreset(MaterialType::Gravel);

    heightfield_.setDefaultMaterial(currentStage_.primaryMaterial);

    std::vector<HeightSample1D> samples;
    constexpr double dx { 1.0 };
    for (double x { -20.0 }; x <= 520.0; x += dx) {
        // Progressive steady incline with terrace step-ups
        const double baseSlope { (x / 500.0) * 6.0 };
        const double terrace { 0.7 * std::sin(2.0 * kPi * x / 35.0) };
        samples.push_back(HeightSample1D{ x, baseSlope + terrace });
    }
    heightfield_.setSamples(samples);
}

void CourseManager::generateSecretFortress()
{
    currentStage_.stageIndex = 8;
    currentStage_.title = "Montsoft Secret Fortress";
    currentStage_.biome = "Military Base";
    currentStage_.totalLength = 500.0;
    currentStage_.parTime = 95.0;
    currentStage_.primaryMaterial = SurfaceMaterial::getPreset(MaterialType::Concrete);

    heightfield_.setDefaultMaterial(currentStage_.primaryMaterial);

    std::vector<HeightSample1D> samples;
    constexpr double dx { 1.0 };
    for (double x { -20.0 }; x <= 520.0; x += dx) {
        double y { 0.0 };

        // Blast ditch at [120, 132]
        if (x >= 120.0 && x <= 132.0) {
            y = -2.5;
        }
        // Bunker ramp at [260, 275]
        else if (x >= 260.0 && x <= 275.0) {
            y = 1.8;
        }
        // Flat boss arena at [430, 520]
        else if (x >= 430.0) {
            y = 0.0;
        } else {
            y = 0.2 * std::sin(2.0 * kPi * x / 25.0);
        }

        samples.push_back(HeightSample1D{ x, y });
    }
    heightfield_.setSamples(samples);
}

} // namespace montsoft_offroad
