#include "apps/MontsoftOffroad/InputController.h"

namespace montsoft_offroad {

InputController::InputController() noexcept
{
    reset();
}

void InputController::handleEvent(const SDL_Event& event) noexcept
{
    if (event.type == SDL_KEYDOWN && event.key.repeat == 0) {
        handleKeyDown(event.key.keysym.sym);
    } else if (event.type == SDL_KEYUP) {
        handleKeyUp(event.key.keysym.sym);
    } else if (event.type == SDL_CONTROLLERBUTTONDOWN) {
        switch (event.cbutton.button) {
        case SDL_CONTROLLER_BUTTON_A:
            jumpPressed_ = true;
            truckInput_.jump = true;
            break;
        case SDL_CONTROLLER_BUTTON_X:
            firePrimaryPressed_ = true;
            truckInput_.firePrimary = true;
            break;
        case SDL_CONTROLLER_BUTTON_Y:
        case SDL_CONTROLLER_BUTTON_B:
            fireSecondaryPressed_ = true;
            truckInput_.fireSecondary = true;
            break;
        case SDL_CONTROLLER_BUTTON_DPAD_RIGHT:
            throttlePressed_ = true;
            truckInput_.throttle = 1.0;
            break;
        case SDL_CONTROLLER_BUTTON_DPAD_LEFT:
        case SDL_CONTROLLER_BUTTON_DPAD_DOWN:
            brakePressed_ = true;
            truckInput_.brake = 1.0;
            break;
        case SDL_CONTROLLER_BUTTON_DPAD_UP:
            jumpPressed_ = true;
            truckInput_.jump = true;
            break;
        case SDL_CONTROLLER_BUTTON_RIGHTSHOULDER:
            pendingAction_ = SystemAction::CyclePalette;
            break;
        case SDL_CONTROLLER_BUTTON_START:
            pendingAction_ = SystemAction::TogglePause;
            break;
        case SDL_CONTROLLER_BUTTON_BACK:
            pendingAction_ = SystemAction::Quit;
            break;
        default:
            break;
        }
    } else if (event.type == SDL_CONTROLLERBUTTONUP) {
        switch (event.cbutton.button) {
        case SDL_CONTROLLER_BUTTON_A:
            jumpPressed_ = false;
            truckInput_.jump = false;
            break;
        case SDL_CONTROLLER_BUTTON_X:
            firePrimaryPressed_ = false;
            truckInput_.firePrimary = false;
            break;
        case SDL_CONTROLLER_BUTTON_Y:
        case SDL_CONTROLLER_BUTTON_B:
            fireSecondaryPressed_ = false;
            truckInput_.fireSecondary = false;
            break;
        case SDL_CONTROLLER_BUTTON_DPAD_RIGHT:
            throttlePressed_ = false;
            if (!throttlePressed_) {
                truckInput_.throttle = 0.0;
            }
            break;
        case SDL_CONTROLLER_BUTTON_DPAD_LEFT:
        case SDL_CONTROLLER_BUTTON_DPAD_DOWN:
            brakePressed_ = false;
            if (!brakePressed_) {
                truckInput_.brake = 0.0;
            }
            break;
        case SDL_CONTROLLER_BUTTON_DPAD_UP:
            jumpPressed_ = false;
            if (!jumpPressed_) {
                truckInput_.jump = false;
            }
            break;
        default:
            break;
        }
    } else if (event.type == SDL_CONTROLLERAXISMOTION) {
        constexpr std::int16_t kDeadzone { 8000 };
        if (event.caxis.axis == SDL_CONTROLLER_AXIS_TRIGGERLEFT) {
            const double val { static_cast<double>(event.caxis.value) / 32767.0 };
            truckInput_.brake = val > 0.1 ? val : (brakePressed_ ? 1.0 : 0.0);
        } else if (event.caxis.axis == SDL_CONTROLLER_AXIS_TRIGGERRIGHT) {
            const double val { static_cast<double>(event.caxis.value) / 32767.0 };
            truckInput_.throttle = val > 0.1 ? val : (throttlePressed_ ? 1.0 : 0.0);
        } else if (event.caxis.axis == SDL_CONTROLLER_AXIS_LEFTX) {
            if (event.caxis.value > kDeadzone) {
                truckInput_.throttle = static_cast<double>(event.caxis.value) / 32767.0;
            } else if (event.caxis.value < -kDeadzone) {
                truckInput_.brake = static_cast<double>(-event.caxis.value) / 32768.0;
            } else {
                if (!throttlePressed_) {
                    truckInput_.throttle = 0.0;
                }
                if (!brakePressed_) {
                    truckInput_.brake = 0.0;
                }
            }
        }
    }
}

void InputController::handleKeyDown(SDL_Keycode key) noexcept
{
    switch (key) {
    case SDLK_RIGHT:
    case SDLK_d:
    case SDLK_w:
        throttlePressed_ = true;
        truckInput_.throttle = 1.0;
        break;
    case SDLK_LEFT:
    case SDLK_a:
    case SDLK_DOWN:
    case SDLK_s:
        brakePressed_ = true;
        truckInput_.brake = 1.0;
        break;
    case SDLK_UP:
    case SDLK_SPACE:
        jumpPressed_ = true;
        truckInput_.jump = true;
        break;
    case SDLK_z:
    case SDLK_j:
    case SDLK_LCTRL:
    case SDLK_RCTRL:
    case SDLK_RETURN:
        firePrimaryPressed_ = true;
        truckInput_.firePrimary = true;
        break;
    case SDLK_x:
    case SDLK_k:
    case SDLK_LALT:
    case SDLK_RALT:
    case SDLK_LSHIFT:
    case SDLK_RSHIFT:
        fireSecondaryPressed_ = true;
        truckInput_.fireSecondary = true;
        break;
    case SDLK_c:
    case SDLK_TAB:
        pendingAction_ = SystemAction::CyclePalette;
        break;
    case SDLK_F1:
        pendingAction_ = SystemAction::SetPaletteMode1;
        break;
    case SDLK_F2:
        pendingAction_ = SystemAction::SetPaletteMode0;
        break;
    case SDLK_F3:
        pendingAction_ = SystemAction::SetPaletteComposite;
        break;
    case SDLK_F4:
        pendingAction_ = SystemAction::SetPaletteEga;
        break;
    case SDLK_p:
        pendingAction_ = SystemAction::TogglePause;
        break;
    case SDLK_ESCAPE:
        pendingAction_ = SystemAction::Quit;
        break;
    default:
        break;
    }
}

void InputController::handleKeyUp(SDL_Keycode key) noexcept
{
    switch (key) {
    case SDLK_RIGHT:
    case SDLK_d:
    case SDLK_w:
        throttlePressed_ = false;
        if (!throttlePressed_) {
            truckInput_.throttle = 0.0;
        }
        break;
    case SDLK_LEFT:
    case SDLK_a:
    case SDLK_DOWN:
    case SDLK_s:
        brakePressed_ = false;
        if (!brakePressed_) {
            truckInput_.brake = 0.0;
        }
        break;
    case SDLK_UP:
    case SDLK_SPACE:
        jumpPressed_ = false;
        if (!jumpPressed_) {
            truckInput_.jump = false;
        }
        break;
    case SDLK_z:
    case SDLK_j:
    case SDLK_LCTRL:
    case SDLK_RCTRL:
    case SDLK_RETURN:
        firePrimaryPressed_ = false;
        if (!firePrimaryPressed_) {
            truckInput_.firePrimary = false;
        }
        break;
    case SDLK_x:
    case SDLK_k:
    case SDLK_LALT:
    case SDLK_RALT:
    case SDLK_LSHIFT:
    case SDLK_RSHIFT:
        fireSecondaryPressed_ = false;
        if (!fireSecondaryPressed_) {
            truckInput_.fireSecondary = false;
        }
        break;
    default:
        break;
    }
}

void InputController::update(double /*dt*/) noexcept
{
    // Continuous analog smoothing hook if needed
}

const vehicle_sim::dynamics::OffroadTruckInput& InputController::getTruckInput() const noexcept
{
    return truckInput_;
}

SystemAction InputController::consumeSystemAction() noexcept
{
    const SystemAction action { pendingAction_ };
    pendingAction_ = SystemAction::None;
    return action;
}

void InputController::setTruckInput(const vehicle_sim::dynamics::OffroadTruckInput& input) noexcept
{
    truckInput_ = input;
}

void InputController::reset() noexcept
{
    truckInput_ = vehicle_sim::dynamics::OffroadTruckInput{};
    pendingAction_ = SystemAction::None;
    throttlePressed_ = false;
    brakePressed_ = false;
    jumpPressed_ = false;
    firePrimaryPressed_ = false;
    fireSecondaryPressed_ = false;
}

} // namespace montsoft_offroad
