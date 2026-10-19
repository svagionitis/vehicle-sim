#include "apps/MontsoftOffroad/CgaPaletteRenderer.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>

namespace montsoft_offroad {

namespace {

    [[nodiscard]] inline int colorDistSq(ColorRgba c1, ColorRgba c2) noexcept
    {
        const int dr { static_cast<int>(c1.r) - static_cast<int>(c2.r) };
        const int dg { static_cast<int>(c1.g) - static_cast<int>(c2.g) };
        const int db { static_cast<int>(c1.b) - static_cast<int>(c2.b) };
        return dr * dr + dg * dg + db * db;
    }

} // namespace

CgaPaletteRenderer::CgaPaletteRenderer() noexcept
{
    populatePalette();
    rebuildLut();
    clear();
}

bool CgaPaletteRenderer::initialize(SDL_Renderer* renderer) noexcept
{
    if (renderer == nullptr) {
        return false;
    }

    SDL_Texture* tex { SDL_CreateTexture(
        renderer, SDL_PIXELFORMAT_RGBA8888, SDL_TEXTUREACCESS_STREAMING, kNativeWidth, kNativeHeight) };

    if (tex == nullptr) {
        return false;
    }

    streamingTexture_.reset(tex);
    return true;
}

void CgaPaletteRenderer::setPaletteMode(PaletteMode mode) noexcept
{
    if (mode_ != mode) {
        mode_ = mode;
        populatePalette();
        rebuildLut();
    }
}

PaletteMode CgaPaletteRenderer::getPaletteMode() const noexcept
{
    return mode_;
}

void CgaPaletteRenderer::cyclePaletteMode() noexcept
{
    switch (mode_) {
    case PaletteMode::CgaPalette1:
        setPaletteMode(PaletteMode::CgaPalette0);
        break;
    case PaletteMode::CgaPalette0:
        setPaletteMode(PaletteMode::CompositeNtsc);
        break;
    case PaletteMode::CompositeNtsc:
        setPaletteMode(PaletteMode::EnhancedEga);
        break;
    case PaletteMode::EnhancedEga:
        setPaletteMode(PaletteMode::CgaPalette1);
        break;
    }
}

void CgaPaletteRenderer::clear(ColorRgba color) noexcept
{
    const ColorRgba quantized { quantizeColor(color) };
    const std::uint32_t packed { quantized.toUint32() };
    backbuffer_.fill(packed);
}

void CgaPaletteRenderer::setPixel(int x, int y, ColorRgba color) noexcept
{
    if (x >= 0 && x < kNativeWidth && y >= 0 && y < kNativeHeight) {
        const std::size_t index { static_cast<std::size_t>(y * kNativeWidth + x) };
        backbuffer_[index] = quantizeColor(color).toUint32();
    }
}

ColorRgba CgaPaletteRenderer::getPixel(int x, int y) const noexcept
{
    if (x >= 0 && x < kNativeWidth && y >= 0 && y < kNativeHeight) {
        const std::size_t index { static_cast<std::size_t>(y * kNativeWidth + x) };
        return ColorRgba::fromUint32(backbuffer_[index]);
    }
    return ColorRgba { 0, 0, 0, 255 };
}

void CgaPaletteRenderer::drawLine(int x0, int y0, int x1, int y1, ColorRgba color) noexcept
{
    const ColorRgba qColor { quantizeColor(color) };
    const std::uint32_t packed { qColor.toUint32() };

    int dx { std::abs(x1 - x0) };
    int dy { -std::abs(y1 - y0) };
    const int sx { (x0 < x1) ? 1 : -1 };
    const int sy { (y0 < y1) ? 1 : -1 };
    int err { dx + dy };

    while (true) {
        if (x0 >= 0 && x0 < kNativeWidth && y0 >= 0 && y0 < kNativeHeight) {
            backbuffer_[static_cast<std::size_t>(y0 * kNativeWidth + x0)] = packed;
        }

        if (x0 == x1 && y0 == y1) {
            break;
        }

        const int e2 { 2 * err };
        if (e2 >= dy) {
            err += dy;
            x0 += sx;
        }
        if (e2 <= dx) {
            err += dx;
            y0 += sy;
        }
    }
}

void CgaPaletteRenderer::drawRect(int x, int y, int width, int height, ColorRgba color) noexcept
{
    if (width <= 0 || height <= 0) {
        return;
    }

    const int right { x + width - 1 };
    const int bottom { y + height - 1 };

    drawLine(x, y, right, y, color);
    drawLine(right, y, right, bottom, color);
    drawLine(right, bottom, x, bottom, color);
    drawLine(x, bottom, x, y, color);
}

void CgaPaletteRenderer::fillRect(int x, int y, int width, int height, ColorRgba color) noexcept
{
    if (width <= 0 || height <= 0) {
        return;
    }

    const int xStart { std::max(0, x) };
    const int yStart { std::max(0, y) };
    const int xEnd { std::min(kNativeWidth, x + width) };
    const int yEnd { std::min(kNativeHeight, y + height) };

    if (xStart >= xEnd || yStart >= yEnd) {
        return;
    }

    const ColorRgba qColor { quantizeColor(color) };
    const std::uint32_t packed { qColor.toUint32() };

    for (int cy { yStart }; cy < yEnd; ++cy) {
        const std::size_t rowOffset { static_cast<std::size_t>(cy * kNativeWidth) };
        for (int cx { xStart }; cx < xEnd; ++cx) {
            backbuffer_[rowOffset + static_cast<std::size_t>(cx)] = packed;
        }
    }
}

void CgaPaletteRenderer::drawCircle(int cx, int cy, int radius, ColorRgba color) noexcept
{
    if (radius <= 0) {
        setPixel(cx, cy, color);
        return;
    }

    int x { radius };
    int y { 0 };
    int err { 0 };

    while (x >= y) {
        setPixel(cx + x, cy + y, color);
        setPixel(cx + y, cy + x, color);
        setPixel(cx - y, cy + x, color);
        setPixel(cx - x, cy + y, color);
        setPixel(cx - x, cy - y, color);
        setPixel(cx - y, cy - x, color);
        setPixel(cx + y, cy - x, color);
        setPixel(cx + x, cy - y, color);

        if (err <= 0) {
            y += 1;
            err += 2 * y + 1;
        }
        if (err > 0) {
            x -= 1;
            err -= 2 * x + 1;
        }
    }
}

void CgaPaletteRenderer::fillCircle(int cx, int cy, int radius, ColorRgba color) noexcept
{
    if (radius <= 0) {
        setPixel(cx, cy, color);
        return;
    }

    for (int dy { -radius }; dy <= radius; ++dy) {
        const int dx { static_cast<int>(std::round(std::sqrt(radius * radius - dy * dy))) };
        const int py { cy + dy };
        if (py >= 0 && py < kNativeHeight) {
            const int xStart { std::clamp(cx - dx, 0, kNativeWidth - 1) };
            const int xEnd { std::clamp(cx + dx, 0, kNativeWidth - 1) };
            const std::uint32_t packed { quantizeColor(color).toUint32() };
            const std::size_t rowOffset { static_cast<std::size_t>(py * kNativeWidth) };
            for (int px { xStart }; px <= xEnd; ++px) {
                backbuffer_[rowOffset + static_cast<std::size_t>(px)] = packed;
            }
        }
    }
}

void CgaPaletteRenderer::blitSprite(
    int dstX, int dstY, int width, int height, const ColorRgba* spritePixels, ColorRgba colorKey) noexcept
{
    if (spritePixels == nullptr || width <= 0 || height <= 0) {
        return;
    }

    for (int sy { 0 }; sy < height; ++sy) {
        const int py { dstY + sy };
        if (py < 0 || py >= kNativeHeight) {
            continue;
        }

        const std::size_t rowOffset { static_cast<std::size_t>(py * kNativeWidth) };
        for (int sx { 0 }; sx < width; ++sx) {
            const int px { dstX + sx };
            if (px < 0 || px >= kNativeWidth) {
                continue;
            }

            const ColorRgba srcColor { spritePixels[sy * width + sx] };
            if (srcColor == colorKey) {
                continue;
            }

            backbuffer_[rowOffset + static_cast<std::size_t>(px)] = quantizeColor(srcColor).toUint32();
        }
    }
}

namespace {

    // 5x7 font ROM for ASCII characters 32 (' ') to 95 ('_')
    constexpr std::uint8_t kFont5x7[64][7] = {
        // 32 ' '
        { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
        // 33 '!'
        { 0x04, 0x04, 0x04, 0x04, 0x04, 0x00, 0x04 },
        // 34 '"'
        { 0x0A, 0x0A, 0x0A, 0x00, 0x00, 0x00, 0x00 },
        // 35 '#'
        { 0x0A, 0x0A, 0x1F, 0x0A, 0x1F, 0x0A, 0x0A },
        // 36 '$'
        { 0x04, 0x0F, 0x14, 0x0E, 0x05, 0x1E, 0x04 },
        // 37 '%'
        { 0x18, 0x19, 0x02, 0x04, 0x08, 0x13, 0x03 },
        // 38 '&'
        { 0x08, 0x14, 0x14, 0x08, 0x15, 0x12, 0x0D },
        // 39 '\''
        { 0x0C, 0x04, 0x08, 0x00, 0x00, 0x00, 0x00 },
        // 40 '('
        { 0x02, 0x04, 0x08, 0x08, 0x08, 0x04, 0x02 },
        // 41 ')'
        { 0x08, 0x04, 0x02, 0x02, 0x02, 0x04, 0x08 },
        // 42 '*'
        { 0x00, 0x04, 0x15, 0x0E, 0x15, 0x04, 0x00 },
        // 43 '+'
        { 0x00, 0x04, 0x04, 0x1F, 0x04, 0x04, 0x00 },
        // 44 ','
        { 0x00, 0x00, 0x00, 0x00, 0x0C, 0x04, 0x08 },
        // 45 '-'
        { 0x00, 0x00, 0x00, 0x1F, 0x00, 0x00, 0x00 },
        // 46 '.'
        { 0x00, 0x00, 0x00, 0x00, 0x00, 0x0C, 0x0C },
        // 47 '/'
        { 0x01, 0x02, 0x04, 0x08, 0x10, 0x00, 0x00 },
        // 48 '0'
        { 0x0E, 0x11, 0x13, 0x15, 0x19, 0x11, 0x0E },
        // 49 '1'
        { 0x04, 0x0C, 0x04, 0x04, 0x04, 0x04, 0x0E },
        // 50 '2'
        { 0x0E, 0x11, 0x01, 0x02, 0x04, 0x08, 0x1F },
        // 51 '3'
        { 0x1F, 0x02, 0x04, 0x02, 0x01, 0x11, 0x0E },
        // 52 '4'
        { 0x02, 0x06, 0x0A, 0x12, 0x1F, 0x02, 0x02 },
        // 53 '5'
        { 0x1F, 0x10, 0x1E, 0x01, 0x01, 0x11, 0x0E },
        // 54 '6'
        { 0x06, 0x08, 0x10, 0x1E, 0x11, 0x11, 0x0E },
        // 55 '7'
        { 0x1F, 0x01, 0x02, 0x04, 0x08, 0x08, 0x08 },
        // 56 '8'
        { 0x0E, 0x11, 0x11, 0x0E, 0x11, 0x11, 0x0E },
        // 57 '9'
        { 0x0E, 0x11, 0x11, 0x0F, 0x01, 0x02, 0x0C },
        // 58 ':'
        { 0x00, 0x0C, 0x0C, 0x00, 0x0C, 0x0C, 0x00 },
        // 59 ';'
        { 0x00, 0x0C, 0x0C, 0x00, 0x0C, 0x04, 0x08 },
        // 60 '<'
        { 0x02, 0x04, 0x08, 0x10, 0x08, 0x04, 0x02 },
        // 61 '='
        { 0x00, 0x1F, 0x00, 0x1F, 0x00, 0x00, 0x00 },
        // 62 '>'
        { 0x08, 0x04, 0x02, 0x01, 0x02, 0x04, 0x08 },
        // 63 '?'
        { 0x0E, 0x11, 0x01, 0x02, 0x04, 0x00, 0x04 },
        // 64 '@'
        { 0x0E, 0x11, 0x01, 0x0D, 0x15, 0x15, 0x0E },
        // 65 'A'
        { 0x0E, 0x11, 0x11, 0x1F, 0x11, 0x11, 0x11 },
        // 66 'B'
        { 0x1E, 0x11, 0x11, 0x1E, 0x11, 0x11, 0x1E },
        // 67 'C'
        { 0x0E, 0x11, 0x10, 0x10, 0x10, 0x11, 0x0E },
        // 68 'D'
        { 0x1C, 0x12, 0x11, 0x11, 0x11, 0x12, 0x1C },
        // 69 'E'
        { 0x1F, 0x10, 0x10, 0x1E, 0x10, 0x10, 0x1F },
        // 70 'F'
        { 0x1F, 0x10, 0x10, 0x1E, 0x10, 0x10, 0x10 },
        // 71 'G'
        { 0x0E, 0x11, 0x10, 0x17, 0x11, 0x11, 0x0F },
        // 72 'H'
        { 0x11, 0x11, 0x11, 0x1F, 0x11, 0x11, 0x11 },
        // 73 'I'
        { 0x0E, 0x04, 0x04, 0x04, 0x04, 0x04, 0x0E },
        // 74 'J'
        { 0x07, 0x02, 0x02, 0x02, 0x02, 0x12, 0x0C },
        // 75 'K'
        { 0x11, 0x12, 0x14, 0x18, 0x14, 0x12, 0x11 },
        // 76 'L'
        { 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x1F },
        // 77 'M'
        { 0x11, 0x1B, 0x15, 0x15, 0x11, 0x11, 0x11 },
        // 78 'N'
        { 0x11, 0x11, 0x19, 0x15, 0x13, 0x11, 0x11 },
        // 79 'O'
        { 0x0E, 0x11, 0x11, 0x11, 0x11, 0x11, 0x0E },
        // 80 'P'
        { 0x1E, 0x11, 0x11, 0x1E, 0x10, 0x10, 0x10 },
        // 81 'Q'
        { 0x0E, 0x11, 0x11, 0x11, 0x15, 0x12, 0x0D },
        // 82 'R'
        { 0x1E, 0x11, 0x11, 0x1E, 0x14, 0x12, 0x11 },
        // 83 'S'
        { 0x0E, 0x11, 0x10, 0x0E, 0x01, 0x11, 0x0E },
        // 84 'T'
        { 0x1F, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04 },
        // 85 'U'
        { 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x0E },
        // 86 'V'
        { 0x11, 0x11, 0x11, 0x11, 0x11, 0x0A, 0x04 },
        // 87 'W'
        { 0x11, 0x11, 0x11, 0x15, 0x15, 0x15, 0x0A },
        // 88 'X'
        { 0x11, 0x11, 0x0A, 0x04, 0x0A, 0x11, 0x11 },
        // 89 'Y'
        { 0x11, 0x11, 0x0A, 0x04, 0x04, 0x04, 0x04 },
        // 90 'Z'
        { 0x1F, 0x01, 0x02, 0x04, 0x08, 0x10, 0x1F },
        // 91 '['
        { 0x0E, 0x08, 0x08, 0x08, 0x08, 0x08, 0x0E },
        // 92 '\\'
        { 0x10, 0x08, 0x04, 0x02, 0x01, 0x00, 0x00 },
        // 93 ']'
        { 0x0E, 0x02, 0x02, 0x02, 0x02, 0x02, 0x0E },
        // 94 '^'
        { 0x04, 0x0A, 0x11, 0x00, 0x00, 0x00, 0x00 },
        // 95 '_'
        { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1F }
    };

} // namespace

void CgaPaletteRenderer::drawChar(int x, int y, char c, ColorRgba color) noexcept
{
    // Normalize lowercase characters to uppercase
    char norm { c };
    if (norm >= 'a' && norm <= 'z') {
        norm = static_cast<char>(norm - 'a' + 'A');
    }

    if (norm < 32 || norm > 95) {
        return;
    }

    const std::size_t charIndex { static_cast<std::size_t>(norm - 32) };
    const auto& rows { kFont5x7[charIndex] };
    const ColorRgba qColor { quantizeColor(color) };

    for (int row { 0 }; row < 7; ++row) {
        const int py { y + row };
        if (py < 0 || py >= kNativeHeight) {
            continue;
        }

        const std::uint8_t rowBits { rows[row] };
        for (int col { 0 }; col < 5; ++col) {
            const int px { x + col };
            if (px < 0 || px >= kNativeWidth) {
                continue;
            }

            // Check if col bit (bit 4 - col) is set
            if ((rowBits & (1U << (4 - col))) != 0U) {
                setPixel(px, py, qColor);
            }
        }
    }
}

void CgaPaletteRenderer::drawText(int x, int y, const std::string& text, ColorRgba color) noexcept
{
    int curX { x };
    for (const char c : text) {
        if (c == '\n') {
            curX = x;
            y += 8;
            continue;
        }
        drawChar(curX, y, c, color);
        curX += 6; // 5 pixels width + 1 pixel inter-character spacing
    }
}

int CgaPaletteRenderer::getTextWidth(const std::string& text) const noexcept
{
    if (text.empty()) {
        return 0;
    }

    int maxWidth { 0 };
    int curWidth { 0 };
    for (const char c : text) {
        if (c == '\n') {
            maxWidth = std::max(maxWidth, curWidth);
            curWidth = 0;
        } else {
            curWidth += 6;
        }
    }
    return std::max(maxWidth, curWidth);
}

ColorRgba CgaPaletteRenderer::quantizeColor(ColorRgba in) const noexcept
{
    const std::size_t lutIndex { static_cast<std::size_t>(((in.r >> 3) << 10) | ((in.g >> 3) << 5) | (in.b >> 3)) };
    if (lutIndex < quantLut_.size()) {
        return quantLut_[lutIndex];
    }

    if (activePalette_.empty()) {
        return in;
    }

    ColorRgba bestColor { activePalette_[0] };
    int bestDist { colorDistSq(in, bestColor) };

    for (std::size_t i { 1 }; i < activePalette_.size(); ++i) {
        const int dist { colorDistSq(in, activePalette_[i]) };
        if (dist < bestDist) {
            bestDist = dist;
            bestColor = activePalette_[i];
        }
    }

    return bestColor;
}

void CgaPaletteRenderer::present(SDL_Renderer* renderer, int winWidth, int winHeight) noexcept
{
    if (renderer == nullptr || !streamingTexture_) {
        return;
    }

    SDL_UpdateTexture(
        streamingTexture_.get(), nullptr, backbuffer_.data(), kNativeWidth * static_cast<int>(sizeof(std::uint32_t)));

    const ViewportRect vp { computeViewport(winWidth, winHeight) };

    SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);
    SDL_RenderClear(renderer);

    const SDL_Rect dstRect { vp.x, vp.y, vp.width, vp.height };
    SDL_RenderCopy(renderer, streamingTexture_.get(), nullptr, &dstRect);
    SDL_RenderPresent(renderer);
}

ViewportRect CgaPaletteRenderer::computeViewport(int winWidth, int winHeight) noexcept
{
    if (winWidth <= 0 || winHeight <= 0) {
        return ViewportRect { 0, 0, kNativeWidth, kNativeHeight };
    }

    const double scaleX { static_cast<double>(winWidth) / static_cast<double>(kNativeWidth) };
    const double scaleY { static_cast<double>(winHeight) / static_cast<double>(kNativeHeight) };
    const double scale { std::max(1.0, std::min(scaleX, scaleY)) };

    const int dstWidth { static_cast<int>(static_cast<double>(kNativeWidth) * scale) };
    const int dstHeight { static_cast<int>(static_cast<double>(kNativeHeight) * scale) };
    const int dstX { (winWidth - dstWidth) / 2 };
    const int dstY { (winHeight - dstHeight) / 2 };

    return ViewportRect { dstX, dstY, dstWidth, dstHeight };
}

const std::uint32_t* CgaPaletteRenderer::getRawBuffer() const noexcept
{
    return backbuffer_.data();
}

const std::vector<ColorRgba>& CgaPaletteRenderer::getActivePalette() const noexcept
{
    return activePalette_;
}

void CgaPaletteRenderer::populatePalette() noexcept
{
    activePalette_.clear();

    switch (mode_) {
    case PaletteMode::CgaPalette1:
        // High Intensity Mode 1: Black, Cyan, Magenta, White
        activePalette_ = {
            ColorRgba { 0, 0, 0, 255 }, // Black
            ColorRgba { 85, 255, 255, 255 }, // Cyan
            ColorRgba { 255, 85, 255, 255 }, // Magenta
            ColorRgba { 255, 255, 255, 255 } // White
        };
        break;

    case PaletteMode::CgaPalette0:
        // High Intensity Mode 0: Black, Green, Red, Yellow
        activePalette_ = {
            ColorRgba { 0, 0, 0, 255 }, // Black
            ColorRgba { 85, 255, 85, 255 }, // Green
            ColorRgba { 255, 85, 85, 255 }, // Red
            ColorRgba { 255, 255, 85, 255 } // Yellow
        };
        break;

    case PaletteMode::CompositeNtsc:
        // 16 Simulated NTSC Composite Artifact Colors
        activePalette_ = {
            ColorRgba { 0, 0, 0, 255 }, // 0: Black
            ColorRgba { 0, 0, 170, 255 }, // 1: Blue
            ColorRgba { 0, 170, 0, 255 }, // 2: Green
            ColorRgba { 0, 170, 170, 255 }, // 3: Cyan
            ColorRgba { 170, 0, 0, 255 }, // 4: Red
            ColorRgba { 170, 0, 170, 255 }, // 5: Magenta
            ColorRgba { 170, 85, 0, 255 }, // 6: Brown
            ColorRgba { 170, 170, 170, 255 }, // 7: Light Gray
            ColorRgba { 85, 85, 85, 255 }, // 8: Dark Gray
            ColorRgba { 85, 85, 255, 255 }, // 9: Bright Blue
            ColorRgba { 85, 255, 85, 255 }, // 10: Bright Green
            ColorRgba { 85, 255, 255, 255 }, // 11: Bright Cyan
            ColorRgba { 255, 85, 85, 255 }, // 12: Bright Red
            ColorRgba { 255, 85, 255, 255 }, // 13: Bright Magenta
            ColorRgba { 255, 255, 85, 255 }, // 14: Yellow
            ColorRgba { 255, 255, 255, 255 } // 15: Bright White
        };
        break;

    case PaletteMode::EnhancedEga:
        // Enhanced 16-Color Retro Desert EGA
        activePalette_ = {
            ColorRgba { 0, 0, 0, 255 }, // Black
            ColorRgba { 0, 0, 170, 255 }, // Deep Blue
            ColorRgba { 0, 170, 0, 255 }, // Sage Green
            ColorRgba { 0, 170, 170, 255 }, // Desert Cyan
            ColorRgba { 170, 0, 0, 255 }, // Rust Red
            ColorRgba { 170, 0, 170, 255 }, // Plum
            ColorRgba { 170, 85, 0, 255 }, // Desert Sand / Brown
            ColorRgba { 170, 170, 170, 255 }, // Sandstone Gray
            ColorRgba { 85, 85, 85, 255 }, // Basalt Charcoal
            ColorRgba { 85, 85, 255, 255 }, // Sky Blue
            ColorRgba { 85, 255, 85, 255 }, // Light Lime
            ColorRgba { 85, 255, 255, 255 }, // Neon Cyan
            ColorRgba { 255, 85, 85, 255 }, // Terracotta
            ColorRgba { 255, 85, 255, 255 }, // Neon Magenta
            ColorRgba { 255, 255, 85, 255 }, // Dune Ochre
            ColorRgba { 255, 255, 255, 255 } // Pure White
        };
        break;
    }
}

void CgaPaletteRenderer::rebuildLut() noexcept
{
    if (activePalette_.empty()) {
        return;
    }

    for (std::size_t r { 0 }; r < 32; ++r) {
        const std::uint8_t realR { static_cast<std::uint8_t>((r * 255) / 31) };
        for (std::size_t g { 0 }; g < 32; ++g) {
            const std::uint8_t realG { static_cast<std::uint8_t>((g * 255) / 31) };
            for (std::size_t b { 0 }; b < 32; ++b) {
                const std::uint8_t realB { static_cast<std::uint8_t>((b * 255) / 31) };
                const ColorRgba sample { realR, realG, realB, 255 };

                ColorRgba bestColor { activePalette_[0] };
                int bestDist { colorDistSq(sample, bestColor) };

                for (std::size_t i { 1 }; i < activePalette_.size(); ++i) {
                    const int dist { colorDistSq(sample, activePalette_[i]) };
                    if (dist < bestDist) {
                        bestDist = dist;
                        bestColor = activePalette_[i];
                    }
                }

                const std::size_t idx { (r << 10) | (g << 5) | b };
                quantLut_[idx] = bestColor;
            }
        }
    }
}

} // namespace montsoft_offroad
