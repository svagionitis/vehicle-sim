#include "apps/MotionLab/TelemetryHud.h"

#include <cctype>
#include <cstdio>

namespace motion_lab {

std::uint16_t TelemetryHud::getGlyph(char c) noexcept
{
    const char upper { static_cast<char>(std::toupper(static_cast<unsigned char>(c))) };

    switch (upper) {
    case '0': return 7u | (5u << 3) | (5u << 6) | (5u << 9) | (7u << 12);
    case '1': return 2u | (3u << 3) | (2u << 6) | (2u << 9) | (7u << 12);
    case '2': return 7u | (4u << 3) | (7u << 6) | (1u << 9) | (7u << 12);
    case '3': return 7u | (4u << 3) | (7u << 6) | (4u << 9) | (7u << 12);
    case '4': return 5u | (5u << 3) | (7u << 6) | (4u << 9) | (4u << 12);
    case '5': return 7u | (1u << 3) | (7u << 6) | (4u << 9) | (7u << 12);
    case '6': return 7u | (1u << 3) | (7u << 6) | (5u << 9) | (7u << 12);
    case '7': return 7u | (4u << 3) | (2u << 6) | (2u << 9) | (2u << 12);
    case '8': return 7u | (5u << 3) | (7u << 6) | (5u << 9) | (7u << 12);
    case '9': return 7u | (5u << 3) | (7u << 6) | (4u << 9) | (7u << 12);

    case 'A': return 7u | (5u << 3) | (7u << 6) | (5u << 9) | (5u << 12);
    case 'B': return 7u | (5u << 3) | (7u << 6) | (5u << 9) | (7u << 12);
    case 'C': return 7u | (1u << 3) | (1u << 6) | (1u << 9) | (7u << 12);
    case 'D': return 3u | (5u << 3) | (5u << 6) | (5u << 9) | (3u << 12);
    case 'E': return 7u | (1u << 3) | (7u << 6) | (1u << 9) | (7u << 12);
    case 'F': return 7u | (1u << 3) | (7u << 6) | (1u << 9) | (1u << 12);
    case 'G': return 7u | (1u << 3) | (5u << 6) | (5u << 9) | (7u << 12);
    case 'H': return 5u | (5u << 3) | (7u << 6) | (5u << 9) | (5u << 12);
    case 'I': return 7u | (2u << 3) | (2u << 6) | (2u << 9) | (7u << 12);
    case 'J': return 4u | (4u << 3) | (4u << 6) | (5u << 9) | (7u << 12);
    case 'K': return 5u | (5u << 3) | (3u << 6) | (5u << 9) | (5u << 12);
    case 'L': return 1u | (1u << 3) | (1u << 6) | (1u << 9) | (7u << 12);
    case 'M': return 5u | (7u << 3) | (5u << 6) | (5u << 9) | (5u << 12);
    case 'N': return 5u | (7u << 3) | (7u << 6) | (5u << 9) | (5u << 12);
    case 'O': return 7u | (5u << 3) | (5u << 6) | (5u << 9) | (7u << 12);
    case 'P': return 7u | (5u << 3) | (7u << 6) | (1u << 9) | (1u << 12);
    case 'Q': return 7u | (5u << 3) | (5u << 6) | (7u << 9) | (6u << 12);
    case 'R': return 7u | (5u << 3) | (7u << 6) | (3u << 9) | (5u << 12);
    case 'S': return 7u | (1u << 3) | (7u << 6) | (4u << 9) | (7u << 12);
    case 'T': return 7u | (2u << 3) | (2u << 6) | (2u << 9) | (2u << 12);
    case 'U': return 5u | (5u << 3) | (5u << 6) | (5u << 9) | (7u << 12);
    case 'V': return 5u | (5u << 3) | (5u << 6) | (5u << 9) | (2u << 12);
    case 'W': return 5u | (5u << 3) | (5u << 6) | (7u << 9) | (5u << 12);
    case 'X': return 5u | (5u << 3) | (2u << 6) | (5u << 9) | (5u << 12);
    case 'Y': return 5u | (5u << 3) | (7u << 6) | (2u << 9) | (2u << 12);
    case 'Z': return 7u | (4u << 3) | (2u << 6) | (1u << 9) | (7u << 12);

    case ':': return 0u | (2u << 3) | (0u << 6) | (2u << 9) | (0u << 12);
    case '-': return 0u | (0u << 3) | (7u << 6) | (0u << 9) | (0u << 12);
    case '.': return 0u | (0u << 3) | (0u << 6) | (0u << 9) | (2u << 12);
    case '(': return 2u | (1u << 3) | (1u << 6) | (1u << 9) | (2u << 12);
    case ')': return 2u | (4u << 3) | (4u << 6) | (4u << 9) | (2u << 12);
    case '/': return 4u | (4u << 3) | (2u << 6) | (1u << 9) | (1u << 12);
    case '%': return 5u | (4u << 3) | (2u << 6) | (1u << 9) | (5u << 12);
    default:  return 0u;
    }
}

void TelemetryHud::drawText(SDL_Renderer* renderer,
                           const std::string& text,
                           int x,
                           int y,
                           int scale,
                           SDL_Color color) noexcept
{
    if (renderer == nullptr || scale <= 0) {
        return;
    }

    SDL_SetRenderDrawColor(renderer, color.r, color.g, color.b, color.a);
    int cursorX { x };

    for (const char ch : text) {
        if (ch == ' ') {
            cursorX += 4 * scale;
            continue;
        }

        const std::uint16_t glyph { getGlyph(ch) };
        for (int row { 0 }; row < 5; ++row) {
            for (int col { 0 }; col < 3; ++col) {
                const int bitIndex { row * 3 + col };
                if ((glyph & (1u << bitIndex)) != 0u) {
                    SDL_Rect pixelRect {
                        cursorX + col * scale,
                        y + row * scale,
                        scale,
                        scale
                    };
                    SDL_RenderFillRect(renderer, &pixelRect);
                }
            }
        }
        cursorX += 4 * scale;
    }
}

void TelemetryHud::render(SDL_Renderer* renderer,
                         const TelemetrySnapshot& snapshot,
                         int screenWidth,
                         int screenHeight) noexcept
{
    if (renderer == nullptr) {
        return;
    }

    const SDL_Color whiteColor { 255, 255, 255, 220 };
    const SDL_Color cyanColor { 140, 230, 197, 255 };
    const SDL_Color goldColor { 212, 171, 113, 255 };
    const SDL_Color dimmedColor { 120, 140, 150, 180 };

    // 1. Top-Left Live State Metrics (Pat Kerr style stats)
    char buffer[128] { };
    std::snprintf(buffer, sizeof(buffer), "SPEED   %.2F M/S (%.1F MPH)", snapshot.speedMps, snapshot.speedMph);
    drawText(renderer, buffer, 24, 20, 2, whiteColor);

    std::snprintf(buffer, sizeof(buffer), "SPIN    %.2F RAD/S", snapshot.angularVelocity);
    drawText(renderer, buffer, 24, 38, 2, whiteColor);

    std::snprintf(buffer, sizeof(buffer), "STEP    %.3F MS", snapshot.stepDurationMs);
    drawText(renderer, buffer, 24, 56, 2, whiteColor);

    std::snprintf(buffer, sizeof(buffer), "DROPPED %.3F S", snapshot.droppedSeconds);
    drawText(renderer, buffer, 24, 74, 2, whiteColor);

    std::snprintf(buffer, sizeof(buffer), "FPS     %.1F", snapshot.fps);
    drawText(renderer, buffer, 24, 92, 2, whiteColor);

    // 2. Bottom Caption Bar
    std::string caption { };
    caption += snapshot.barrierOn ? "BARRIER ON / " : "BARRIER OFF / ";
    caption += snapshot.gravityOn ? "GRAVITY ON / " : "GRAVITY OFF / ";
    caption += snapshot.deadZoneOn ? "DEAD ZONE ON / " : "DEAD ZONE OFF / ";
    caption += snapshot.speedZoomOn ? "SPEED ZOOM ON" : "SPEED ZOOM OFF";
    drawText(renderer, caption, 24, screenHeight - 34, 2, goldColor);

    // 3. Bottom Version Tag
    drawText(renderer, "MOTION LAB 1996 REMASTER (CAR)", screenWidth - 420, screenHeight - 34, 2, dimmedColor);

    // 4. Input Indicator HUD (if toggled on via 'H')
    if (snapshot.inputHudVisible) {
        const int hudY { screenHeight - 90 };
        int hudX { screenWidth - 480 };

        auto drawIndicator = [&](const char* label, bool active) {
            const int boxWidth { 84 };
            const int boxHeight { 32 };
            SDL_Rect boxRect { hudX, hudY, boxWidth, boxHeight };

            if (active) {
                SDL_SetRenderDrawColor(renderer, 140, 230, 197, 80);
                SDL_RenderFillRect(renderer, &boxRect);
                SDL_SetRenderDrawColor(renderer, 140, 230, 197, 255);
                SDL_RenderDrawRect(renderer, &boxRect);
                drawText(renderer, label, hudX + 8, hudY + 10, 2, cyanColor);
            } else {
                SDL_SetRenderDrawColor(renderer, 30, 40, 48, 120);
                SDL_RenderFillRect(renderer, &boxRect);
                SDL_SetRenderDrawColor(renderer, 60, 80, 90, 200);
                SDL_RenderDrawRect(renderer, &boxRect);
                drawText(renderer, label, hudX + 8, hudY + 10, 2, dimmedColor);
            }
            hudX += boxWidth + 8;
        };

        drawIndicator("LEFT", snapshot.steerLeft);
        drawIndicator("RIGHT", snapshot.steerRight);
        drawIndicator("ACCEL", snapshot.throttleForward);
        drawIndicator("DECEL", snapshot.throttleReverse);
        drawIndicator("HBRAKE", snapshot.handbrake);
    }

    // 5. Pause Banner
    if (snapshot.isPaused) {
        SDL_Rect pauseBox { screenWidth / 2 - 120, screenHeight / 2 - 40, 240, 80 };
        SDL_SetRenderDrawColor(renderer, 8, 9, 10, 220);
        SDL_RenderFillRect(renderer, &pauseBox);
        SDL_SetRenderDrawColor(renderer, 212, 171, 113, 255);
        SDL_RenderDrawRect(renderer, &pauseBox);
        drawText(renderer, "SIMULATION PAUSED", screenWidth / 2 - 100, screenHeight / 2 - 15, 2, goldColor);
        drawText(renderer, "PRESS P TO RESUME", screenWidth / 2 - 95, screenHeight / 2 + 10, 2, whiteColor);
    }
}

} // namespace motion_lab
