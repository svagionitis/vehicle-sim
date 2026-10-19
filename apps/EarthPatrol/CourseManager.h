#pragma once

#include <cstddef>
#include <vector>

#include "libs/VehicleSimCore/terrain/Heightfield1D.h"

namespace earth_patrol {

/// @struct CheckpointInfo
/// @brief Checkpoint marker position and attainment state.
struct CheckpointInfo {
    char letter { 'A' }; ///< Section milestone letter (A through E).
    double worldX { 100.0 }; ///< Longitudinal world location in meters.
    bool reached { false }; ///< Flag indicating if rover has crossed checkpoint.
};

/// @struct CourseConfig
/// @brief Configuration settings for course progression, milestones, and scoring.
struct CourseConfig {
    double totalLength { 500.0 }; ///< Total stage course length in meters.
    double segmentLength { 100.0 }; ///< Distance between consecutive checkpoints in meters.
    double parTime { 60.0 }; ///< Target completion time in seconds.
    int checkpointBonus { 1000 }; ///< Bonus score points awarded per checkpoint reached.
    int stageClearBonus { 5000 }; ///< Bonus score points awarded upon reaching Point E.
};

/// @class CourseManager
/// @brief Manages procedural heightfield terrain synthesis, checkpoints, and stage pacing.
/// @details Synthesizes continuous 1D heightfield contours with Earth undulating hills,
/// craters, and sinkholes, while tracking checkpoint progress (A through E), remaining time, and score.
class CourseManager {
public:
    /// @brief Default constructor initializing standard 500-meter 5-checkpoint course.
    CourseManager();

    /// @brief Parameterized constructor using custom course configuration.
    /// @param[in] config Course and scoring parameters.
    explicit CourseManager(const CourseConfig& config);

    /// @brief Builds a procedural terrestrial course for the specified biome.
    /// @param[in] biomeIndex Biome identifier (0: Highway Ruins, 1: Desert, 2: Alpine).
    void buildCourse(int biomeIndex);

    /// @brief Updates checkpoint progression and countdown timer.
    /// @param[in] roverWorldX Current longitudinal position of rover in meters.
    /// @param[in] dt Timestep duration in seconds.
    void update(double roverWorldX, double dt) noexcept;

    /// @brief Resets stage progress, timer, and checkpoints.
    void reset() noexcept;

    /// @brief Gets active terrain heightfield.
    /// @return Constant reference to Heightfield1D.
    [[nodiscard]] const vehicle_sim::terrain::Heightfield1D& getHeightfield() const noexcept;

    /// @brief Gets mutable active terrain heightfield.
    /// @return Mutable reference to Heightfield1D.
    [[nodiscard]] vehicle_sim::terrain::Heightfield1D& getHeightfield() noexcept;

    /// @brief Gets letter of the next upcoming checkpoint milestone.
    /// @return Upcoming milestone letter ('A' to 'E').
    [[nodiscard]] char getCurrentCheckpoint() const noexcept;

    /// @brief Computes remaining distance to upcoming checkpoint in meters.
    /// @return Remaining distance in meters.
    [[nodiscard]] double getDistanceToNext() const noexcept;

    /// @brief Computes normalized stage progress along the course in [0.0, 1.0].
    /// @return Normalized progress scalar.
    [[nodiscard]] double getProgressFraction() const noexcept;

    /// @brief Checks if stage finish line (Point E) has been reached.
    /// @return True if stage is complete.
    [[nodiscard]] bool isStageComplete() const noexcept;

    /// @brief Gets remaining countdown time in seconds.
    /// @return Remaining seconds.
    [[nodiscard]] double getRemainingTime() const noexcept;

    /// @brief Gets active total score.
    /// @return Total accumulated points.
    [[nodiscard]] int getScore() const noexcept;

    /// @brief Adds points to total score.
    /// @param[in] points Points to add.
    void addScore(int points) noexcept;

    /// @brief Gets registered checkpoints list.
    /// @return Vector of CheckpointInfo.
    [[nodiscard]] const std::vector<CheckpointInfo>& getCheckpoints() const noexcept;

    /// @brief Gets active course configuration.
    /// @return Constant reference to CourseConfig.
    [[nodiscard]] const CourseConfig& getConfig() const noexcept;

private:
    CourseConfig config_ {};
    vehicle_sim::terrain::Heightfield1D heightfield_ {};
    std::vector<CheckpointInfo> checkpoints_ {};

    double roverX_ { 0.0 };
    double remainingTime_ { 60.0 };
    int score_ { 0 };
    bool stageComplete_ { false };
    std::size_t nextCheckpointIdx_ { 0 };
};

} // namespace earth_patrol
