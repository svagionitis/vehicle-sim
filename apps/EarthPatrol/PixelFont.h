#pragma once

#include <cstdint>
#include <string>

#include <SDL2/SDL.h>

namespace earth_patrol {

/// @class PixelFont
/// @brief Lightweight procedural 3x5 arcade bitmap font renderer.
/// @details Rasterizes retro alphanumeric characters without external font dependencies
/// or file assets, supporting configurable integer scaling and colors.
class PixelFont {
public:
    /// @brief Draws a single text string at the target screen coordinates.
    /// @param[in] renderer Active SDL hardware renderer pointer.
    /// @param[in] text Null-terminated text string to draw.
    /// @param[in] x Screen X coordinate in pixels for the top-left origin.
    /// @param[in] y Screen Y coordinate in pixels for the top-left origin.
    /// @param[in] scale Pixel magnification factor (1 = 3x5, 2 = 6x10, etc.).
    /// @param[in] color RGBA tint color for the font pixels.
    static void drawText(
        SDL_Renderer* renderer, const std::string& text, int x, int y, int scale, SDL_Color color) noexcept;

    /// @brief Calculates the horizontal pixel width required to render a string.
    /// @param[in] text Text string to measure.
    /// @param[in] scale Pixel magnification factor.
    /// @return Total rendered width in pixels.
    [[nodiscard]] static int getTextWidth(const std::string& text, int scale) noexcept;

private:
    /// @brief Retrieves the 15-bit bitmap pattern for a given character.
    /// @param[in] c ASCII character to encode.
    /// @return 15-bit packed representation of 3x5 pixels.
    [[nodiscard]] static std::uint16_t getGlyph(char c) noexcept;
};

} // namespace earth_patrol
