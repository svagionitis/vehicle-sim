#pragma once

#include <cstdint>
#include <SDL.h>

#include "libs/VehicleSimCore/dynamics/OffroadTruckDynamics.h"

namespace montsoft_offroad {

/// @enum SystemAction
/// @brief Discrete game-level commands triggered by input actions.
enum class SystemAction : std::uint8_t {
    None,
    CyclePalette,
    CycleZoom,
    SetPaletteMode1,
    SetPaletteMode0,
    SetPaletteComposite,
    SetPaletteEga,
    TogglePause,
    Quit
};

/// @class InputController
/// @brief Polls and maps hardware keyboard and gamepad states into vehicle commands.
/// @details Decouples SDL input event polling from GameApp loop, ensuring deterministic
/// state handling and edge-triggered action latching.
class InputController final {
public:
    /// @brief Default constructor.
    InputController() noexcept;

    /// @brief Destructor.
    ~InputController() noexcept = default;

    /// @brief Processes an incoming SDL input event.
    /// @param[in] event Reference to SDL event union.
    void handleEvent(const SDL_Event& event) noexcept;

    /// @brief Updates continuous analog smoothing and resets edge-triggered flags.
    /// @param[in] dt Frame delta time in seconds.
    void update(double dt) noexcept;

    /// @brief Gets active continuous driving input commands for OffroadTruckDynamics.
    /// @return Constant reference to active OffroadTruckInput.
    [[nodiscard]] const vehicle_sim::dynamics::OffroadTruckInput& getTruckInput() const noexcept;

    /// @brief Consumes and clears any latched system-level action.
    /// @return The latched SystemAction, or SystemAction::None if none pending.
    [[nodiscard]] SystemAction consumeSystemAction() noexcept;

    /// @brief Manually injects truck driving inputs (for testing and scripting).
    /// @param[in] input Control input commands.
    void setTruckInput(const vehicle_sim::dynamics::OffroadTruckInput& input) noexcept;

    /// @brief Resets all inputs and latched actions to neutral defaults.
    void reset() noexcept;

private:
    void handleKeyDown(SDL_Keycode key) noexcept;
    void handleKeyUp(SDL_Keycode key) noexcept;

    vehicle_sim::dynamics::OffroadTruckInput truckInput_ {};
    SystemAction pendingAction_ { SystemAction::None };
    bool throttlePressed_ { false };
    bool brakePressed_ { false };
    bool jumpPressed_ { false };
    bool firePrimaryPressed_ { false };
    bool fireSecondaryPressed_ { false };
};

} // namespace montsoft_offroad
