#include "apps/EarthPatrol/PixelFont.h"

#include <cctype>
#include <cstdint>

namespace earth_patrol {

std::uint16_t PixelFont::getGlyph(char c) noexcept
{
    const char upper { static_cast<char>(std::toupper(static_cast<unsigned char>(c))) };

    switch (upper) {
    case '0':
        return 7u | (5u << 3) | (5u << 6) | (5u << 9) | (7u << 12);
    case '1':
        return 2u | (3u << 3) | (2u << 6) | (2u << 9) | (7u << 12);
    case '2':
        return 7u | (4u << 3) | (7u << 6) | (1u << 9) | (7u << 12);
    case '3':
        return 7u | (4u << 3) | (7u << 6) | (4u << 9) | (7u << 12);
    case '4':
        return 5u | (5u << 3) | (7u << 6) | (4u << 9) | (4u << 12);
    case '5':
        return 7u | (1u << 3) | (7u << 6) | (4u << 9) | (7u << 12);
    case '6':
        return 7u | (1u << 3) | (7u << 6) | (5u << 9) | (7u << 12);
    case '7':
        return 7u | (4u << 3) | (2u << 6) | (2u << 9) | (2u << 12);
    case '8':
        return 7u | (5u << 3) | (7u << 6) | (5u << 9) | (7u << 12);
    case '9':
        return 7u | (5u << 3) | (7u << 6) | (4u << 9) | (7u << 12);

    case 'A':
        return 7u | (5u << 3) | (7u << 6) | (5u << 9) | (5u << 12);
    case 'B':
        return 7u | (5u << 3) | (7u << 6) | (5u << 9) | (7u << 12);
    case 'C':
        return 7u | (1u << 3) | (1u << 6) | (1u << 9) | (7u << 12);
    case 'D':
        return 3u | (5u << 3) | (5u << 6) | (5u << 9) | (3u << 12);
    case 'E':
        return 7u | (1u << 3) | (7u << 6) | (1u << 9) | (7u << 12);
    case 'F':
        return 7u | (1u << 3) | (7u << 6) | (1u << 9) | (1u << 12);
    case 'G':
        return 7u | (1u << 3) | (5u << 6) | (5u << 9) | (7u << 12);
    case 'H':
        return 5u | (5u << 3) | (7u << 6) | (5u << 9) | (5u << 12);
    case 'I':
        return 7u | (2u << 3) | (2u << 6) | (2u << 9) | (7u << 12);
    case 'J':
        return 4u | (4u << 3) | (4u << 6) | (5u << 9) | (7u << 12);
    case 'K':
        return 5u | (5u << 3) | (3u << 6) | (5u << 9) | (5u << 12);
    case 'L':
        return 1u | (1u << 3) | (1u << 6) | (1u << 9) | (7u << 12);
    case 'M':
        return 5u | (7u << 3) | (5u << 6) | (5u << 9) | (5u << 12);
    case 'N':
        return 5u | (7u << 3) | (7u << 6) | (5u << 9) | (5u << 12);
    case 'O':
        return 7u | (5u << 3) | (5u << 6) | (5u << 9) | (7u << 12);
    case 'P':
        return 7u | (5u << 3) | (7u << 6) | (1u << 9) | (1u << 12);
    case 'Q':
        return 7u | (5u << 3) | (5u << 6) | (7u << 9) | (6u << 12);
    case 'R':
        return 7u | (5u << 3) | (7u << 6) | (3u << 9) | (5u << 12);
    case 'S':
        return 7u | (1u << 3) | (7u << 6) | (4u << 9) | (7u << 12);
    case 'T':
        return 7u | (2u << 3) | (2u << 6) | (2u << 9) | (2u << 12);
    case 'U':
        return 5u | (5u << 3) | (5u << 6) | (5u << 9) | (7u << 12);
    case 'V':
        return 5u | (5u << 3) | (5u << 6) | (5u << 9) | (2u << 12);
    case 'W':
        return 5u | (5u << 3) | (5u << 6) | (7u << 9) | (5u << 12);
    case 'X':
        return 5u | (5u << 3) | (2u << 6) | (5u << 9) | (5u << 12);
    case 'Y':
        return 5u | (5u << 3) | (7u << 6) | (2u << 9) | (2u << 12);
    case 'Z':
        return 7u | (4u << 3) | (2u << 6) | (1u << 9) | (7u << 12);

    case ':':
        return 0u | (2u << 3) | (0u << 6) | (2u << 9) | (0u << 12);
    case '-':
        return 0u | (0u << 3) | (7u << 6) | (0u << 9) | (0u << 12);
    case '.':
        return 0u | (0u << 3) | (0u << 6) | (0u << 9) | (2u << 12);
    case '!':
        return 2u | (2u << 3) | (2u << 6) | (0u << 9) | (2u << 12);
    case '/':
        return 4u | (4u << 3) | (2u << 6) | (1u << 9) | (1u << 12);
    case '+':
        return 0u | (2u << 3) | (7u << 6) | (2u << 9) | (0u << 12);
    default:
        return 0u;
    }
}

int PixelFont::getTextWidth(const std::string& text, int scale) noexcept
{
    if (text.empty() || scale <= 0) {
        return 0;
    }
    // Each glyph is 3 cols + 1 col spacing = 4 cols * scale
    return static_cast<int>(text.length()) * 4 * scale;
}

void PixelFont::drawText(
    SDL_Renderer* renderer, const std::string& text, int x, int y, int scale, SDL_Color color) noexcept
{
    if (renderer == nullptr || scale <= 0 || text.empty()) {
        return;
    }

    SDL_SetRenderDrawColor(renderer, color.r, color.g, color.b, color.a);

    int curX { x };
    for (const char c : text) {
        const std::uint16_t glyph { getGlyph(c) };

        for (int row { 0 }; row < 5; ++row) {
            for (int col { 0 }; col < 3; ++col) {
                const int bitIdx { (row * 3) + col };
                if (((glyph >> bitIdx) & 1u) != 0u) {
                    SDL_Rect pixelRect { curX + (col * scale), y + (row * scale), scale, scale };
                    SDL_RenderFillRect(renderer, &pixelRect);
                }
            }
        }
        curX += 4 * scale;
    }
}

} // namespace earth_patrol
