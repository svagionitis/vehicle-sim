#include "apps/MontsoftOffroad/HighScoreManager.h"

#include <algorithm>
#include <fstream>
#include <sstream>

namespace montsoft_offroad {

HighScoreManager::HighScoreManager() noexcept
{
    resetToDefaults();
    static_cast<void>(loadScores());
}

void HighScoreManager::resetToDefaults() noexcept
{
    records_[0] = HighScoreRecord{ { 'M', 'T', 'V', '\0' }, 50000, 8 };
    records_[1] = HighScoreRecord{ { 'M', 'O', 'N', '\0' }, 42500, 8 };
    records_[2] = HighScoreRecord{ { 'A', 'C', 'E', '\0' }, 35000, 7 };
    records_[3] = HighScoreRecord{ { 'C', 'G', 'A', '\0' }, 28000, 6 };
    records_[4] = HighScoreRecord{ { 'D', 'O', 'S', '\0' }, 22000, 5 };
    records_[5] = HighScoreRecord{ { 'P', 'I', 'T', '\0' }, 18000, 4 };
    records_[6] = HighScoreRecord{ { 'B', 'E', 'K', '\0' }, 14000, 3 };
    records_[7] = HighScoreRecord{ { 'R', 'U', 'T', '\0' }, 10000, 2 };
    records_[8] = HighScoreRecord{ { 'W', 'N', 'G', '\0' },  7500, 1 };
    records_[9] = HighScoreRecord{ { 'S', 'I', 'M', '\0' },  5000, 1 };
}

bool HighScoreManager::loadScores() noexcept
{
    std::ifstream file{ filePath_ };
    if (!file.is_open()) {
        return false;
    }

    std::string line {};
    std::size_t count { 0 };
    std::array<HighScoreRecord, kMaxRecords> tempRecords {};

    while (count < kMaxRecords && std::getline(file, line)) {
        if (line.empty() || line[0] == '#') {
            continue;
        }

        std::istringstream iss{ line };
        std::string init {};
        int score { 0 };
        int stage { 1 };

        if (iss >> init >> score >> stage) {
            HighScoreRecord rec {};
            rec.initials[0] = (init.size() > 0) ? init[0] : ' ';
            rec.initials[1] = (init.size() > 1) ? init[1] : ' ';
            rec.initials[2] = (init.size() > 2) ? init[2] : ' ';
            rec.initials[3] = '\0';
            rec.score = std::max(0, score);
            rec.stageReached = std::clamp(stage, 1, 8);
            tempRecords[count++] = rec;
        }
    }

    if (count == kMaxRecords) {
        records_ = tempRecords;
        return true;
    }

    return false;
}

bool HighScoreManager::saveScores() const noexcept
{
    std::ofstream file{ filePath_ };
    if (!file.is_open()) {
        return false;
    }

    file << "# Montsoft Offroad (1988) High Score Leaderboard\n";
    for (const auto& rec : records_) {
        file << rec.initials.data() << ' ' << rec.score << ' ' << rec.stageReached << '\n';
    }

    return true;
}

bool HighScoreManager::isHighScore(int score) const noexcept
{
    return score > records_[kMaxRecords - 1].score;
}

int HighScoreManager::submitScore(const std::array<char, 3>& initials, int score, int stage) noexcept
{
    if (!isHighScore(score)) {
        return -1;
    }

    // Find insertion index
    std::size_t insertIdx { kMaxRecords };
    for (std::size_t i { 0 }; i < kMaxRecords; ++i) {
        if (score > records_[i].score) {
            insertIdx = i;
            break;
        }
    }

    if (insertIdx >= kMaxRecords) {
        return -1;
    }

    // Shift lower records down
    for (std::size_t i { kMaxRecords - 1 }; i > insertIdx; --i) {
        records_[i] = records_[i - 1];
    }

    // Insert new record
    HighScoreRecord newRec {};
    newRec.initials[0] = initials[0];
    newRec.initials[1] = initials[1];
    newRec.initials[2] = initials[2];
    newRec.initials[3] = '\0';
    newRec.score = score;
    newRec.stageReached = std::clamp(stage, 1, 8);
    records_[insertIdx] = newRec;

    static_cast<void>(saveScores());

    return static_cast<int>(insertIdx);
}

const std::array<HighScoreRecord, HighScoreManager::kMaxRecords>& HighScoreManager::getRecords() const noexcept
{
    return records_;
}

} // namespace montsoft_offroad
