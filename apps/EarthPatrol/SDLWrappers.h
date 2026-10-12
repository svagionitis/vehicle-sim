#pragma once

#include <memory>
#include <string>
#include <string_view>

#include <SDL2/SDL.h>

namespace earth_patrol {

/// @struct SDLWindowDeleter
/// @brief Custom RAII deleter for SDL_Window resource pointers.
struct SDLWindowDeleter {
    /// @brief Releases SDL_Window memory.
    /// @param[in] window Pointer to SDL_Window.
    void operator()(SDL_Window* window) const noexcept
    {
        if (window != nullptr) {
            SDL_DestroyWindow(window);
        }
    }
};

/// @struct SDLRendererDeleter
/// @brief Custom RAII deleter for SDL_Renderer resource pointers.
struct SDLRendererDeleter {
    /// @brief Releases SDL_Renderer memory.
    /// @param[in] renderer Pointer to SDL_Renderer.
    void operator()(SDL_Renderer* renderer) const noexcept
    {
        if (renderer != nullptr) {
            SDL_DestroyRenderer(renderer);
        }
    }
};

/// @struct SDLTextureDeleter
/// @brief Custom RAII deleter for SDL_Texture resource pointers.
struct SDLTextureDeleter {
    /// @brief Releases SDL_Texture memory.
    /// @param[in] texture Pointer to SDL_Texture.
    void operator()(SDL_Texture* texture) const noexcept
    {
        if (texture != nullptr) {
            SDL_DestroyTexture(texture);
        }
    }
};

/// @struct SDLControllerDeleter
/// @brief Custom RAII deleter for SDL_GameController resource pointers.
struct SDLControllerDeleter {
    /// @brief Closes SDL_GameController device handle.
    /// @param[in] controller Pointer to SDL_GameController.
    void operator()(SDL_GameController* controller) const noexcept
    {
        if (controller != nullptr) {
            SDL_GameControllerClose(controller);
        }
    }
};

/// @brief Unique pointer alias for managed SDL_Window.
using UniqueWindow = std::unique_ptr<SDL_Window, SDLWindowDeleter>;

/// @brief Unique pointer alias for managed SDL_Renderer.
using UniqueRenderer = std::unique_ptr<SDL_Renderer, SDLRendererDeleter>;

/// @brief Unique pointer alias for managed SDL_Texture.
using UniqueTexture = std::unique_ptr<SDL_Texture, SDLTextureDeleter>;

/// @brief Unique pointer alias for managed SDL_GameController.
using UniqueController = std::unique_ptr<SDL_GameController, SDLControllerDeleter>;

/// @class SDLContext
/// @brief RAII manager for SDL subsystem lifecycle initialization and teardown.
class SDLContext final {
public:
    /// @brief Initializes SDL video, timer, gamecontroller, and event subsystems.
    /// @param[in] flags SDL initialization flags (default: video, events, timer, controller).
    explicit SDLContext(Uint32 flags = SDL_INIT_VIDEO | SDL_INIT_EVENTS | SDL_INIT_TIMER | SDL_INIT_GAMECONTROLLER)
    {
        if (SDL_Init(flags) != 0) {
            initialized_ = false;
        } else {
            initialized_ = true;
        }
    }

    /// @brief Cleans up SDL subsystems.
    ~SDLContext() noexcept
    {
        if (initialized_) {
            SDL_Quit();
        }
    }

    SDLContext(const SDLContext&) = delete;
    SDLContext& operator=(const SDLContext&) = delete;
    SDLContext(SDLContext&&) noexcept = default;
    SDLContext& operator=(SDLContext&&) noexcept = default;

    /// @brief Queries whether SDL subsystems were successfully initialized.
    /// @return True if initialized.
    [[nodiscard]] bool isInitialized() const noexcept
    {
        return initialized_;
    }

    /// @brief Retrieves the last SDL error string.
    /// @return Last error message.
    [[nodiscard]] static std::string_view getLastError() noexcept
    {
        return SDL_GetError();
    }

private:
    bool initialized_ { false };
};

} // namespace earth_patrol
