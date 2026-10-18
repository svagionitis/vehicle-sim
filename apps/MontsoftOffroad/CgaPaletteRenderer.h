#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

#include <SDL2/SDL.h>
#include "apps/MontsoftOffroad/SDLWrappers.h"

namespace montsoft_offroad {

/// @enum PaletteMode
/// @brief Active retro color display mode.
enum class PaletteMode : std::uint8_t {
    CgaPalette1,   ///< High-intensity Cyan, Magenta, White, Black.
    CgaPalette0,   ///< High-intensity Green, Red, Yellow, Black.
    CompositeNtsc, ///< 16-color composite video artifact emulation.
    EnhancedEga    ///< 16-color retro desert EGA palette.
};

/// @struct ColorRgba
/// @brief Packed 32-bit RGBA color representation.
struct ColorRgba {
    std::uint8_t r { 0 };
    std::uint8_t g { 0 };
    std::uint8_t b { 0 };
    std::uint8_t a { 255 };

    /// @brief Pack to 32-bit unsigned integer in RGBA8888 byte format.
    /// @return 32-bit packed color.
    [[nodiscard]] constexpr std::uint32_t toUint32() const noexcept
    {
        return (static_cast<std::uint32_t>(r) << 24) |
               (static_cast<std::uint32_t>(g) << 16) |
               (static_cast<std::uint32_t>(b) << 8)  |
               static_cast<std::uint32_t>(a);
    }

    /// @brief Construct ColorRgba from 32-bit RGBA8888 value.
    /// @param[in] value Packed 32-bit color.
    /// @return Decoded ColorRgba.
    [[nodiscard]] static constexpr ColorRgba fromUint32(std::uint32_t value) noexcept
    {
        return ColorRgba{
            static_cast<std::uint8_t>((value >> 24) & 0xFF),
            static_cast<std::uint8_t>((value >> 16) & 0xFF),
            static_cast<std::uint8_t>((value >> 8) & 0xFF),
            static_cast<std::uint8_t>(value & 0xFF)
        };
    }

    /// @brief Equality comparison operator.
    [[nodiscard]] constexpr bool operator==(const ColorRgba& other) const noexcept
    {
        return r == other.r && g == other.g && b == other.b && a == other.a;
    }

    /// @brief Inequality comparison operator.
    [[nodiscard]] constexpr bool operator!=(const ColorRgba& other) const noexcept
    {
        return !(*this == other);
    }
};

/// @struct ViewportRect
/// @brief Scaled target destination rectangle maintaining aspect ratio.
struct ViewportRect {
    int x { 0 };
    int y { 0 };
    int width { 0 };
    int height { 0 };
};

/// @class CgaPaletteRenderer
/// @brief Software rasterizer and CGA/EGA color quantizer outputting to a streaming SDL texture.
/// @details Maintains a 320x200 pixel virtual backbuffer, provides basic 2D drawing primitives,
/// maps colors to authentic retro palettes, and renders letterboxed/pillarboxed to the window.
class CgaPaletteRenderer final {
public:
    static constexpr int kNativeWidth { 320 };
    static constexpr int kNativeHeight { 200 };
    static constexpr std::size_t kPixelCount { 320 * 200 };

    /// @brief Default constructor initializing Mode 1.
    CgaPaletteRenderer() noexcept;

    /// @brief Destructor.
    ~CgaPaletteRenderer() noexcept = default;

    CgaPaletteRenderer(const CgaPaletteRenderer&) = delete;
    CgaPaletteRenderer& operator=(const CgaPaletteRenderer&) = delete;
    CgaPaletteRenderer(CgaPaletteRenderer&&) noexcept = default;
    CgaPaletteRenderer& operator=(CgaPaletteRenderer&&) noexcept = default;

    /// @brief Initializes streaming texture on target renderer.
    /// @param[in] renderer Pointer to active SDL_Renderer.
    /// @return True on success, false on texture creation failure.
    [[nodiscard]] bool initialize(SDL_Renderer* renderer) noexcept;

    /// @brief Sets active palette mode and rebuilds fast lookup tables.
    /// @param[in] mode PaletteMode enum value.
    void setPaletteMode(PaletteMode mode) noexcept;

    /// @brief Gets active palette mode.
    /// @return Active PaletteMode.
    [[nodiscard]] PaletteMode getPaletteMode() const noexcept;

    /// @brief Cycles forward to next available palette mode.
    void cyclePaletteMode() noexcept;

    /// @brief Clears virtual backbuffer to specific color.
    /// @param[in] color Fill color.
    void clear(ColorRgba color = { 0, 0, 0, 255 }) noexcept;

    /// @brief Draws a single pixel into the virtual backbuffer.
    /// @param[in] x Coordinate X in [0, 319].
    /// @param[in] y Coordinate Y in [0, 199].
    /// @param[in] color Pixel RGBA color.
    void setPixel(int x, int y, ColorRgba color) noexcept;

    /// @brief Gets pixel color from virtual backbuffer.
    /// @param[in] x Coordinate X in [0, 319].
    /// @param[in] y Coordinate Y in [0, 199].
    /// @return ColorRgba value at coordinate.
    [[nodiscard]] ColorRgba getPixel(int x, int y) const noexcept;

    /// @brief Draws a line using Bresenham's algorithm.
    /// @param[in] x0 Start X coordinate.
    /// @param[in] y0 Start Y coordinate.
    /// @param[in] x1 End X coordinate.
    /// @param[in] y1 End Y coordinate.
    /// @param[in] color Line color.
    void drawLine(int x0, int y0, int x1, int y1, ColorRgba color) noexcept;

    /// @brief Draws an unfilled rectangle.
    /// @param[in] x Top-left X.
    /// @param[in] y Top-left Y.
    /// @param[in] width Rectangle width.
    /// @param[in] height Rectangle height.
    /// @param[in] color Line color.
    void drawRect(int x, int y, int width, int height, ColorRgba color) noexcept;

    /// @brief Draws a solid filled rectangle.
    /// @param[in] x Top-left X.
    /// @param[in] y Top-left Y.
    /// @param[in] width Rectangle width.
    /// @param[in] height Rectangle height.
    /// @param[in] color Fill color.
    void fillRect(int x, int y, int width, int height, ColorRgba color) noexcept;

    /// @brief Blits a sprite with color key transparency.
    /// @param[in] dstX Target buffer X.
    /// @param[in] dstY Target buffer Y.
    /// @param[in] width Sprite width in pixels.
    /// @param[in] height Sprite height in pixels.
    /// @param[in] spritePixels Pointer to sprite pixel array.
    /// @param[in] colorKey Transparent color key value.
    void blitSprite(int dstX, int dstY, int width, int height,
                    const ColorRgba* spritePixels, ColorRgba colorKey) noexcept;

    /// @brief Quantizes input color against active palette using nearest Euclidean distance.
    /// @param[in] in Color to quantize.
    /// @return Closest ColorRgba in active palette.
    [[nodiscard]] ColorRgba quantizeColor(ColorRgba in) const noexcept;

    /// @brief Updates streaming texture and renders to current SDL renderer.
    /// @param[in] renderer Active SDL_Renderer.
    /// @param[in] winWidth Current window width.
    /// @param[in] winHeight Current window height.
    void present(SDL_Renderer* renderer, int winWidth, int winHeight) noexcept;

    /// @brief Computes aspect-preserving letterbox viewport rectangle.
    /// @param[in] winWidth Current window width in pixels.
    /// @param[in] winHeight Current window height in pixels.
    /// @return ViewportRect containing centered dimensions.
    [[nodiscard]] static ViewportRect computeViewport(int winWidth, int winHeight) noexcept;

    /// @brief Directly accesses raw 320x200 pixel backbuffer.
    /// @return Pointer to array of 64,000 uint32_t RGBA pixels.
    [[nodiscard]] const std::uint32_t* getRawBuffer() const noexcept;

    /// @brief Gets active palette colors array.
    /// @return Vector of ColorRgba entries in current palette.
    [[nodiscard]] const std::vector<ColorRgba>& getActivePalette() const noexcept;

private:
    void rebuildLut() noexcept;
    void populatePalette() noexcept;

    PaletteMode mode_ { PaletteMode::CgaPalette1 };
    std::array<std::uint32_t, kPixelCount> backbuffer_ {};
    UniqueTexture streamingTexture_ { nullptr };
    std::vector<ColorRgba> activePalette_ {};
    std::array<ColorRgba, 32768> quantLut_ {};
};

} // namespace montsoft_offroad
