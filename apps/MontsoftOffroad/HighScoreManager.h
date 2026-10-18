#pragma once

#include <array>
#include <cstddef>
#include <string>

namespace montsoft_offroad {

/// @struct HighScoreRecord
/// @brief High score record entry containing initials, points, and stage reached.
struct HighScoreRecord {
    std::array<char, 4> initials { 'M', 'T', 'V', '\0' }; ///< Player initials + null terminator.
    int score { 10000 };                                    ///< Accumulated game points.
    int stageReached { 1 };                                 ///< Furthest stage index reached (1-8).
};

/// @class HighScoreManager
/// @brief Manages top-10 leaderboard persistence and ranking.
class HighScoreManager final {
public:
    static constexpr std::size_t kMaxRecords { 10 };

    /// @brief Default constructor loading scores from disk or defaults.
    HighScoreManager() noexcept;

    /// @brief Destructor.
    ~HighScoreManager() noexcept = default;

    HighScoreManager(const HighScoreManager&) = delete;
    HighScoreManager& operator=(const HighScoreManager&) = delete;
    HighScoreManager(HighScoreManager&&) noexcept = default;
    HighScoreManager& operator=(HighScoreManager&&) noexcept = default;

    /// @brief Loads high score records from default storage file.
    /// @return True on successful load, false on missing or invalid file.
    bool loadScores() noexcept;

    /// @brief Saves high score records to default storage file.
    /// @return True on successful write, false on file I/O error.
    bool saveScores() const noexcept;

    /// @brief Checks whether the provided score qualifies for the top-10 leaderboard.
    /// @param[in] score Score to evaluate.
    /// @return True if score exceeds at least one leaderboard entry.
    [[nodiscard]] bool isHighScore(int score) const noexcept;

    /// @brief Submits a new score into the leaderboard, shifting lower entries.
    /// @param[in] initials 3-letter player initials.
    /// @param[in] score Final accumulated score.
    /// @param[in] stage Furthest stage reached.
    /// @return Rank index in [0, 9], or -1 if score did not qualify.
    int submitScore(const std::array<char, 3>& initials, int score, int stage) noexcept;

    /// @brief Gets active leaderboard records.
    /// @return Constant reference to array of HighScoreRecord.
    [[nodiscard]] const std::array<HighScoreRecord, kMaxRecords>& getRecords() const noexcept;

    /// @brief Resets leaderboard to historical default shareware records.
    void resetToDefaults() noexcept;

private:
    std::array<HighScoreRecord, kMaxRecords> records_ {};
    std::string filePath_ { "montsoft_scores.dat" };
};

} // namespace montsoft_offroad
