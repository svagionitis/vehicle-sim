#pragma once

#include <array>
#include <cstddef>
#include <string>
#include <vector>

#include "libs/VehicleSimCore/terrain/Heightfield1D.h"
#include "libs/VehicleSimCore/terrain/SurfaceMaterial.h"

namespace montsoft_offroad {

/// @struct CheckpointMarker
/// @brief Checkpoint milestone information and reached status.
struct CheckpointMarker {
    char letter { 'A' };       ///< Milestone letter ('A' through 'E').
    double worldX { 100.0 };   ///< Longitudinal track coordinate in meters.
    int scoreBonus { 1000 };   ///< Score points awarded upon crossing.
    double timeBonus { 15.0 }; ///< Pacing seconds added upon crossing.
    bool reached { false };    ///< Attainment flag.
};

/// @struct StageDescriptor
/// @brief Metadata and physical characteristics for an Offroad campaign stage.
struct StageDescriptor {
    int stageIndex { 1 };                    ///< Stage number (1 through 8).
    std::string title { "Dunes of Mojave" }; ///< Display title of the stage.
    std::string biome { "Desert Dunes" };    ///< Geological biome classification.
    double totalLength { 500.0 };            ///< Total distance from start to Point E in meters.
    double parTime { 75.0 };                 ///< Initial countdown timer allocation in seconds.
    vehicle_sim::terrain::SurfaceMaterial primaryMaterial {}; ///< Dominant ground material.
};

/// @class CourseManager
/// @brief Manages 8-stage terrain heightfield synthesis, checkpoints, and stage progression.
/// @details Generates continuous Heightfield1D contours with procedural and authored obstacles,
/// tracks milestone splits (A-E), remaining time, and stage completion.
class CourseManager final {
public:
    static constexpr std::size_t kTotalStages { 8 };
    static constexpr std::size_t kCheckpointsPerStage { 5 };

    /// @brief Default constructor initializing Stage 1 (Dunes of Mojave).
    CourseManager();

    /// @brief Destructor.
    ~CourseManager() noexcept = default;

    CourseManager(const CourseManager&) = delete;
    CourseManager& operator=(const CourseManager&) = delete;
    CourseManager(CourseManager&&) noexcept = default;
    CourseManager& operator=(CourseManager&&) noexcept = default;

    /// @brief Builds heightfield and checkpoints for the specified stage number.
    /// @param[in] stageIndex Stage index in [1, 8].
    void loadStage(int stageIndex);

    /// @brief Updates vehicle progression along the course and decrements countdown timer.
    /// @param[in] truckX Current longitudinal position of the truck in meters.
    /// @param[in] dt Timestep duration in seconds.
    void update(double truckX, double dt) noexcept;

    /// @brief Resets current stage progression, restoring timer and uncrossing checkpoints.
    void reset() noexcept;

    /// @brief Gets active terrain heightfield.
    /// @return Constant reference to Heightfield1D.
    [[nodiscard]] const vehicle_sim::terrain::Heightfield1D& getHeightfield() const noexcept;

    /// @brief Gets active stage metadata.
    /// @return Constant reference to StageDescriptor.
    [[nodiscard]] const StageDescriptor& getCurrentStage() const noexcept;

    /// @brief Gets active stage number in [1, 8].
    /// @return Current stage index.
    [[nodiscard]] int getCurrentStageIndex() const noexcept;

    /// @brief Gets list of stage checkpoint markers.
    /// @return Constant reference to vector of CheckpointMarker.
    [[nodiscard]] const std::vector<CheckpointMarker>& getCheckpoints() const noexcept;

    /// @brief Gets letter of the next upcoming milestone.
    /// @return Upcoming milestone letter ('A' to 'E').
    [[nodiscard]] char getNextCheckpointLetter() const noexcept;

    /// @brief Calculates remaining distance to the next upcoming checkpoint in meters.
    /// @return Distance in meters.
    [[nodiscard]] double getDistanceToNext() const noexcept;

    /// @brief Gets normalized stage progression in [0.0, 1.0].
    /// @return Fractional progress along the 500m course.
    [[nodiscard]] double getProgressFraction() const noexcept;

    /// @brief Checks if stage finish line (Point E) has been attained.
    /// @return True if stage is complete.
    [[nodiscard]] bool isStageComplete() const noexcept;

    /// @brief Gets remaining countdown time in seconds.
    /// @return Time in seconds.
    [[nodiscard]] double getRemainingTime() const noexcept;

    /// @brief Gets total accumulated score.
    /// @return Current score.
    [[nodiscard]] int getScore() const noexcept;

    /// @brief Adds points to total accumulated score.
    /// @param[in] points Points to add.
    void addScore(int points) noexcept;

private:
    void generateMojaveDunes();
    void generateRedRockArroyo();
    void generateGreatSaltFlats();
    void generateMuddyQuagmire();
    void generateVolcanicBadlands();
    void generateScrapCanyon();
    void generateAlpineRidge();
    void generateSecretFortress();

    void setupCheckpoints(double length);

    StageDescriptor currentStage_ {};
    vehicle_sim::terrain::Heightfield1D heightfield_ {};
    std::vector<CheckpointMarker> checkpoints_ {};
    double truckX_ { 0.0 };
    double remainingTime_ { 75.0 };
    int score_ { 0 };
    bool stageComplete_ { false };
    std::size_t nextCheckpointIdx_ { 0 };
};

} // namespace montsoft_offroad
