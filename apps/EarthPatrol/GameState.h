#pragma once

#include <cstdint>

namespace earth_patrol {

/// @enum GameState
/// @brief Top-level execution state of Earth Patrol arcade loop.
enum class GameState : std::uint8_t {
    TitleScreen, ///< Title banner, controls prompt, awaiting game start.
    Playing, ///< Active simulation gameplay with physics, camera, and hazards.
    Crashed, ///< Post-collision explosion animation and respawn countdown.
    StageClear, ///< Milestone E reached, displaying time bonus and stage advance.
    GameOver ///< All rover lives depleted, showing final score and return prompt.
};

} // namespace earth_patrol
