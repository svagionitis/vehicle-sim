#include "apps/MotionLab/TelemetryHud.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>

namespace motion_lab {

std::uint16_t TelemetryHud::getGlyph(char c) noexcept
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
    case '+':
        return 0u | (2u << 3) | (7u << 6) | (2u << 9) | (0u << 12);
    case '.':
        return 0u | (0u << 3) | (0u << 6) | (0u << 9) | (2u << 12);
    case ',':
        return 0u | (0u << 3) | (0u << 6) | (2u << 9) | (1u << 12);
    case '(':
        return 2u | (1u << 3) | (1u << 6) | (1u << 9) | (2u << 12);
    case ')':
        return 2u | (4u << 3) | (4u << 6) | (4u << 9) | (2u << 12);
    case '[':
        return 3u | (1u << 3) | (1u << 6) | (1u << 9) | (3u << 12);
    case ']':
        return 6u | (4u << 3) | (4u << 6) | (4u << 9) | (6u << 12);
    case '/':
        return 4u | (4u << 3) | (2u << 6) | (1u << 9) | (1u << 12);
    case '%':
        return 5u | (4u << 3) | (2u << 6) | (1u << 9) | (5u << 12);
    default:
        return 0u;
    }
}

void TelemetryHud::drawCircle(SDL_Renderer* renderer, int cx, int cy, int radius, SDL_Color color) noexcept
{
    if (renderer == nullptr || radius <= 0) {
        return;
    }

    SDL_SetRenderDrawColor(renderer, color.r, color.g, color.b, color.a);
    constexpr int kSteps { 32 };
    constexpr double kAngleStep { 6.283185307179586 / static_cast<double>(kSteps) };

    int prevX { cx + radius };
    int prevY { cy };

    for (int i { 1 }; i <= kSteps; ++i) {
        const double angle { static_cast<double>(i) * kAngleStep };
        const int currX { cx + static_cast<int>(std::round(std::cos(angle) * radius)) };
        const int currY { cy + static_cast<int>(std::round(std::sin(angle) * radius)) };
        SDL_RenderDrawLine(renderer, prevX, prevY, currX, currY);
        prevX = currX;
        prevY = currY;
    }
}

void TelemetryHud::drawText(
    SDL_Renderer* renderer, const std::string& text, int x, int y, int scale, SDL_Color color) noexcept
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
                    SDL_Rect pixelRect { cursorX + col * scale, y + row * scale, scale, scale };
                    SDL_RenderFillRect(renderer, &pixelRect);
                }
            }
        }
        cursorX += 4 * scale;
    }
}

void TelemetryHud::renderFrictionCircle(
    SDL_Renderer* renderer, const ExtendedTelemetrySnapshot& extended, int centerX, int centerY, int radius) noexcept
{
    if (renderer == nullptr || radius <= 10) {
        return;
    }

    // Panel background
    const int boxMargin { radius + 16 };
    const SDL_Rect bgRect { centerX - boxMargin, centerY - boxMargin, boxMargin * 2, boxMargin * 2 + 12 };
    SDL_SetRenderDrawColor(renderer, 10, 14, 18, 130);
    SDL_RenderFillRect(renderer, &bgRect);
    SDL_SetRenderDrawColor(renderer, 40, 60, 75, 180);
    SDL_RenderDrawRect(renderer, &bgRect);

    // Title badge
    drawText(renderer, "G-G CIRCLE", centerX - 26, centerY - boxMargin + 4, 1, SDL_Color { 0, 220, 255, 255 });

    // Concentric Reference Circles: 0.5G, 1.0G, 1.5G
    const SDL_Color gridColor { 45, 65, 80, 160 };
    const SDL_Color cyanBoundary { 0, 180, 220, 220 };

    const int r05 { radius / 3 };
    const int r10 { (radius * 2) / 3 };
    const int r15 { radius };

    drawCircle(renderer, centerX, centerY, r05, gridColor);
    drawCircle(renderer, centerX, centerY, r10, cyanBoundary);
    drawCircle(renderer, centerX, centerY, r15, gridColor);

    // Coordinate crosshairs
    SDL_SetRenderDrawColor(renderer, gridColor.r, gridColor.g, gridColor.b, gridColor.a);
    SDL_RenderDrawLine(renderer, centerX - r15, centerY, centerX + r15, centerY);
    SDL_RenderDrawLine(renderer, centerX, centerY - r15, centerX, centerY + r15);

    // Small reference markers
    drawText(renderer, "1G", centerX + r10 - 6, centerY + 2, 1, SDL_Color { 0, 180, 220, 180 });
    drawText(renderer, "+AY", centerX - r15 + 2, centerY - 9, 1, SDL_Color { 140, 180, 200, 160 });
    drawText(renderer, "-AY", centerX + r15 - 16, centerY - 9, 1, SDL_Color { 140, 180, 200, 160 });
    drawText(renderer, "+AX", centerX - 8, centerY - r15 + 2, 1, SDL_Color { 140, 180, 200, 160 });
    drawText(renderer, "-AX", centerX - 8, centerY + r15 - 8, 1, SDL_Color { 140, 180, 200, 160 });

    // Render historical trail cloud (phosphor persistence)
    const std::size_t trailCount { extended.ggTrailCount };
    const std::size_t trailHead { extended.ggTrailHead };
    constexpr double kMaxDisplayG { 1.5 };
    const double gScale { static_cast<double>(radius) / kMaxDisplayG };

    for (std::size_t i { 0U }; i < trailCount; ++i) {
        const std::size_t idx { (trailHead + ExtendedTelemetrySnapshot::kGgTrailCapacity - trailCount + i)
            % ExtendedTelemetrySnapshot::kGgTrailCapacity };
        const auto& pt { extended.ggTrail[idx] };

        // Longitudinal is +Y on screen (up = accel), lateral is +X on screen (left/right)
        const int px { centerX + static_cast<int>(std::round(pt.y * gScale)) };
        const int py { centerY - static_cast<int>(std::round(pt.x * gScale)) };

        const auto alpha { static_cast<std::uint8_t>(40U + (180U * i) / trailCount) };
        SDL_SetRenderDrawColor(renderer, 0, 220, 180, alpha);
        SDL_RenderDrawPoint(renderer, px, py);
    }

    // Operating point: compute traction demand ratio rho
    const double safeMu { std::max(0.1, extended.surfaceFriction) };
    const double rho { std::sqrt(
        std::pow(extended.longitudinalG / safeMu, 2.0) + std::pow(extended.lateralG / safeMu, 2.0)) };

    SDL_Color cursorColor { 0, 255, 136, 255 }; // Neon green
    if (rho > 0.95) {
        cursorColor = SDL_Color { 255, 51, 51, 255 }; // Warning red
    } else if (rho >= 0.70) {
        cursorColor = SDL_Color { 255, 191, 0, 255 }; // Bright amber
    }

    const int curX { centerX + static_cast<int>(std::round(extended.lateralG * gScale)) };
    const int curY { centerY - static_cast<int>(std::round(extended.longitudinalG * gScale)) };

    const SDL_Rect cursorRect { curX - 2, curY - 2, 5, 5 };
    SDL_SetRenderDrawColor(renderer, cursorColor.r, cursorColor.g, cursorColor.b, cursorColor.a);
    SDL_RenderFillRect(renderer, &cursorRect);
    SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);
    SDL_RenderDrawRect(renderer, &cursorRect);

    // Readout line
    char buf[64] { };
    std::snprintf(buf, sizeof(buf), "G:%+4.1F/%+4.1F", extended.longitudinalG, extended.lateralG);
    drawText(renderer, buf, centerX - boxMargin + 6, centerY + radius + 2, 1, SDL_Color { 220, 240, 255, 255 });

    std::snprintf(buf, sizeof(buf), "PK:%3.1F/%3.1F", extended.peakLongG, extended.peakLatG);
    drawText(renderer, buf, centerX - boxMargin + 6, centerY + radius + 10, 1, SDL_Color { 212, 171, 113, 255 });
}

void TelemetryHud::renderWeightTransfer(SDL_Renderer* renderer, const DynamicWheelLoads& loads, int x, int y) noexcept
{
    if (renderer == nullptr) {
        return;
    }

    constexpr int kBoxWidth { 116 };
    constexpr int kBoxHeight { 114 };
    const SDL_Rect bgRect { x, y, kBoxWidth, kBoxHeight };

    SDL_SetRenderDrawColor(renderer, 10, 14, 18, 130);
    SDL_RenderFillRect(renderer, &bgRect);
    SDL_SetRenderDrawColor(renderer, 40, 60, 75, 180);
    SDL_RenderDrawRect(renderer, &bgRect);

    drawText(renderer, "WHEEL LOADS", x + 16, y + 4, 1, SDL_Color { 212, 171, 113, 255 });

    // 4 Bars: FL, FR, RL, RR
    auto drawWheelBar = [&](const char* label, double currentLoad, double staticNominal, int barX, int barY) {
        constexpr int kBarW { 16 };
        constexpr int kBarH { 54 };

        const SDL_Rect barOutline { barX, barY, kBarW, kBarH };
        SDL_SetRenderDrawColor(renderer, 25, 35, 45, 160);
        SDL_RenderFillRect(renderer, &barOutline);
        SDL_SetRenderDrawColor(renderer, 60, 80, 95, 200);
        SDL_RenderDrawRect(renderer, &barOutline);

        // Midpoint marker = 1.0x static load
        const int midY { barY + kBarH / 2 };
        SDL_SetRenderDrawColor(renderer, 120, 140, 160, 160);
        SDL_RenderDrawLine(renderer, barX, midY, barX + kBarW, midY);

        // Fill height proportional to currentLoad / (2.0 * staticNominal)
        const double safeNominal { std::max(10.0, staticNominal) };
        const double ratio { std::clamp(currentLoad / (2.0 * safeNominal), 0.0, 1.0) };
        const int fillH { static_cast<int>(std::round(ratio * static_cast<double>(kBarH))) };
        const int fillY { barY + kBarH - fillH };

        SDL_Color fillColor { 140, 230, 197, 220 };
        if (currentLoad < safeNominal * 0.15) {
            fillColor = SDL_Color { 255, 60, 60, 255 }; // Wheel lift
        } else if (currentLoad > safeNominal * 1.5) {
            fillColor = SDL_Color { 255, 191, 0, 240 }; // Heavy overload
        }

        const SDL_Rect fillRect { barX + 1, fillY, kBarW - 2, fillH };
        SDL_SetRenderDrawColor(renderer, fillColor.r, fillColor.g, fillColor.b, fillColor.a);
        SDL_RenderFillRect(renderer, &fillRect);

        // Label above, load below
        drawText(renderer, label, barX + 3, barY - 9, 1, SDL_Color { 220, 240, 255, 255 });

        char loadBuf[16] { };
        if (currentLoad < safeNominal * 0.10) {
            drawText(renderer, "LIFT", barX - 1, barY + kBarH + 2, 1, SDL_Color { 255, 60, 60, 255 });
        } else {
            std::snprintf(loadBuf, sizeof(loadBuf), "%.1FK", currentLoad * 0.001);
            drawText(renderer, loadBuf, barX - 2, barY + kBarH + 2, 1, SDL_Color { 180, 210, 230, 255 });
        }
    };

    const int barY { y + 26 };
    drawWheelBar("FL", loads.frontLeftLoad, loads.staticFrontWheel, x + 8, barY);
    drawWheelBar("FR", loads.frontRightLoad, loads.staticFrontWheel, x + 34, barY);
    drawWheelBar("RL", loads.rearLeftLoad, loads.staticRearWheel, x + 62, barY);
    drawWheelBar("RR", loads.rearRightLoad, loads.staticRearWheel, x + 88, barY);
}

void TelemetryHud::renderStripChart(SDL_Renderer* renderer, const float* values, std::size_t count,
    std::size_t headIndex, int x, int y, int width, int height, float minY, float maxY, const char* title,
    SDL_Color traceColor) noexcept
{
    if (renderer == nullptr || values == nullptr || width <= 10 || height <= 10) {
        return;
    }

    const SDL_Rect bgRect { x, y, width, height };
    SDL_SetRenderDrawColor(renderer, 10, 14, 18, 130);
    SDL_RenderFillRect(renderer, &bgRect);
    SDL_SetRenderDrawColor(renderer, 35, 50, 65, 180);
    SDL_RenderDrawRect(renderer, &bgRect);

    // Title and limits
    drawText(renderer, title, x + 8, y + 4, 1, traceColor);

    char limitBuf[16] { };
    std::snprintf(limitBuf, sizeof(limitBuf), "%+3.0F", static_cast<double>(maxY));
    drawText(renderer, limitBuf, x + width - 36, y + 4, 1, SDL_Color { 120, 140, 160, 180 });

    std::snprintf(limitBuf, sizeof(limitBuf), "%+3.0F", static_cast<double>(minY));
    drawText(renderer, limitBuf, x + width - 36, y + height - 12, 1, SDL_Color { 120, 140, 160, 180 });

    // Zero-line if range encompasses zero
    if (minY < 0.0f && maxY > 0.0f) {
        const float zeroRatio { (0.0f - minY) / (maxY - minY) };
        const int zeroY { y + height - static_cast<int>(std::round(zeroRatio * static_cast<float>(height))) };
        SDL_SetRenderDrawColor(renderer, 50, 70, 85, 180);
        SDL_RenderDrawLine(renderer, x + 1, zeroY, x + width - 2, zeroY);
    }

    if (count < 2U) {
        return;
    }

    const float range { std::max(0.001f, maxY - minY) };
    constexpr std::size_t kCap { OscilloscopeWindow::kHistoryCapacity };

    SDL_SetRenderDrawColor(renderer, traceColor.r, traceColor.g, traceColor.b, traceColor.a);

    int prevPx { -1 };
    int prevPy { -1 };

    for (std::size_t i { 0U }; i < count; ++i) {
        const std::size_t idx { (headIndex + kCap - count + i) % kCap };
        const float val { std::clamp(values[idx], minY, maxY) };
        const float u { (val - minY) / range };

        const int px { x + 6
            + static_cast<int>((static_cast<double>(i) / static_cast<double>(count - 1U)) * (width - 48)) };
        const int py { y + height - 6 - static_cast<int>(std::round(u * static_cast<float>(height - 14))) };

        if (prevPx >= 0) {
            SDL_RenderDrawLine(renderer, prevPx, prevPy, px, py);
        }
        prevPx = px;
        prevPy = py;
    }
}

void TelemetryHud::render(SDL_Renderer* renderer, const TelemetrySnapshot& snapshot,
    const ExtendedTelemetrySnapshot& extended, int screenWidth, int screenHeight) noexcept
{
    if (renderer == nullptr) {
        return;
    }

    // Clean Screen / Cinematic Mode: all HUD overlays hidden
    if (!snapshot.allHudVisible) {
        if (snapshot.isPaused) {
            const SDL_Rect pauseBox { screenWidth / 2 - 120, screenHeight / 2 - 40, 240, 80 };
            SDL_SetRenderDrawColor(renderer, 8, 9, 10, 220);
            SDL_RenderFillRect(renderer, &pauseBox);
            SDL_SetRenderDrawColor(renderer, 212, 171, 113, 255);
            SDL_RenderDrawRect(renderer, &pauseBox);
            drawText(renderer, "SIMULATION PAUSED", screenWidth / 2 - 100, screenHeight / 2 - 15, 2,
                SDL_Color { 212, 171, 113, 255 });
            drawText(renderer, "PRESS P TO RESUME", screenWidth / 2 - 95, screenHeight / 2 + 10, 2,
                SDL_Color { 255, 255, 255, 220 });
        }
        return;
    }

    SDL_SetRenderDrawBlendMode(renderer, SDL_BLENDMODE_BLEND);

    const SDL_Color whiteColor { 255, 255, 255, 220 };
    const SDL_Color cyanColor { 140, 230, 197, 255 };
    const SDL_Color goldColor { 212, 171, 113, 255 };
    const SDL_Color dimmedColor { 120, 140, 150, 180 };

    // 1. Top-Left Live State Metrics
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

    std::snprintf(buffer, sizeof(buffer), "TERRAIN %s (MU %.2F DRAG %.2F)", snapshot.terrainName.c_str(),
        snapshot.terrainFriction, snapshot.terrainRollingDrag);
    drawText(renderer, buffer, 24, 110, 2, cyanColor);

    std::snprintf(buffer, sizeof(buffer), "VEHICLE %s", snapshot.vehicleName.c_str());
    drawText(renderer, buffer, 24, 128, 2, goldColor);

    if (!snapshot.vehicleDetails.empty()) {
        std::snprintf(buffer, sizeof(buffer), "STATUS  %s", snapshot.vehicleDetails.c_str());
        drawText(renderer, buffer, 24, 146, 2, cyanColor);
    }

    if (snapshot.circuitMode) {
        std::snprintf(buffer, sizeof(buffer), "CIRCUIT LAP %u (SEC %u)  %.1FS", snapshot.completedLaps + 1U,
            snapshot.currentSector, snapshot.currentLapTime);
        drawText(renderer, buffer, 24, 164, 2, cyanColor);

        if (snapshot.bestLapTime > 0.0) {
            std::snprintf(
                buffer, sizeof(buffer), "BEST    %.2FS  LAST %.2FS", snapshot.bestLapTime, snapshot.lastLapTime);
            drawText(renderer, buffer, 24, 182, 2, goldColor);
        }
    } else if (snapshot.slalomMode) {
        drawText(renderer, "COURSE  100M SLALOM ACTIVE [L]", 24, 164, 2, SDL_Color { 255, 140, 0, 255 });
    } else if (snapshot.mooseMode) {
        drawText(renderer, "COURSE  ISO 3888-2 MOOSE TEST ACTIVE [M]", 24, 164, 2, SDL_Color { 0, 240, 255, 255 });
    } else if (snapshot.skidpadMode) {
        drawText(renderer, "COURSE  SKIDPAD CIRCLES ACTIVE [K]", 24, 164, 2, SDL_Color { 0, 255, 200, 255 });
    }

    // 2. Advanced Telemetry HUD Widgets (Consolidated Right Sidebar, keeping center vehicle open)
    if (extended.advancedHudVisible) {
        constexpr int kSidebarW { 228 };
        const int sidebarX { screenWidth - kSidebarW - 16 };
        const int startY { snapshot.circuitMode ? 170 : 20 };

        constexpr int kChartH { 42 };

        // Chart 1: Speed
        renderStripChart(renderer, extended.stripCharts.speedHistory.data(), extended.stripCharts.sampleCount,
            extended.stripCharts.headIndex, sidebarX, startY, kSidebarW, kChartH, 0.0f, 60.0f, "SPEED (M/S)",
            SDL_Color { 0, 240, 255, 255 });

        // Chart 2: Front Slip Angle
        renderStripChart(renderer, extended.stripCharts.frontSlipHistory.data(), extended.stripCharts.sampleCount,
            extended.stripCharts.headIndex, sidebarX, startY + 46, kSidebarW, kChartH, -30.0f, 30.0f,
            "SLIP ANGLE (DEG)", SDL_Color { 255, 191, 0, 255 });

        // Chart 3: Yaw Rate
        renderStripChart(renderer, extended.stripCharts.yawRateHistory.data(), extended.stripCharts.sampleCount,
            extended.stripCharts.headIndex, sidebarX, startY + 92, kSidebarW, kChartH, -180.0f, 180.0f,
            "YAW RATE (DEG/S)", SDL_Color { 140, 230, 197, 255 });

        // G-G Friction Circle (left) and 4-Corner Weight Transfer (right) side-by-side
        const int lowerY { startY + 142 };
        constexpr int kGgRadius { 36 };
        const int ggCenterX { sidebarX + 54 };
        const int ggCenterY { lowerY + 54 };
        renderFrictionCircle(renderer, extended, ggCenterX, ggCenterY, kGgRadius);

        const int wtX { sidebarX + 110 };
        renderWeightTransfer(renderer, extended.wheelLoads, wtX, lowerY);
    }

    // 3. Compact Input Indicator HUD
    if (snapshot.inputHudVisible) {
        const int hudY { screenHeight - 64 };
        int hudX { screenWidth - 264 };

        auto drawIndicator = [&](const char* label, bool active) {
            constexpr int boxWidth { 46 };
            constexpr int boxHeight { 20 };
            const SDL_Rect boxRect { hudX, hudY, boxWidth, boxHeight };

            if (active) {
                SDL_SetRenderDrawColor(renderer, 140, 230, 197, 70);
                SDL_RenderFillRect(renderer, &boxRect);
                SDL_SetRenderDrawColor(renderer, 140, 230, 197, 240);
                SDL_RenderDrawRect(renderer, &boxRect);
                drawText(renderer, label, hudX + 4, hudY + 6, 1, cyanColor);
            } else {
                SDL_SetRenderDrawColor(renderer, 30, 40, 48, 100);
                SDL_RenderFillRect(renderer, &boxRect);
                SDL_SetRenderDrawColor(renderer, 60, 80, 90, 160);
                SDL_RenderDrawRect(renderer, &boxRect);
                drawText(renderer, label, hudX + 4, hudY + 6, 1, dimmedColor);
            }
            hudX += boxWidth + 4;
        };

        drawIndicator("LEFT", snapshot.steerLeft);
        drawIndicator("RIGHT", snapshot.steerRight);
        drawIndicator("ACCEL", snapshot.throttleForward);
        drawIndicator("DECEL", snapshot.throttleReverse);
        drawIndicator("HBRAKE", snapshot.handbrake);
    }

    // 4. Bottom Caption Bar
    std::string caption { "[TAB] HUD / [H] ADV HUD / [SH+H] WIND / [O] AIRFLOW / [I] INPUTS / " };
    caption += snapshot.barrierOn ? "BARRIER ON / " : "BARRIER OFF / ";
    caption += snapshot.gravityOn ? "GRAVITY ON / " : "GRAVITY OFF / ";
    if (snapshot.circuitMode) {
        caption += "[C] CIRCUIT / ";
    } else if (snapshot.slalomMode) {
        caption += "[L] SLALOM / ";
    } else if (snapshot.mooseMode) {
        caption += "[M] MOOSE / ";
    } else if (snapshot.skidpadMode) {
        caption += "[K] SKIDPAD / ";
    }
    caption += "[V] " + snapshot.vehicleName + " / [T] " + snapshot.terrainName;
    drawText(renderer, caption, 24, screenHeight - 26, 1, goldColor);

    // 5. Session Disk Recording Badge
    if (extended.isRecording) {
        char recBuf[64] { };
        std::snprintf(recBuf, sizeof(recBuf), "[REC: %.1FS (%llu)]", extended.recordingDuration,
            static_cast<unsigned long long>(extended.recordedFrames));

        const int recX { screenWidth - 420 };
        const int recY { screenHeight - 26 };
        drawText(renderer, recBuf, recX, recY, 1, SDL_Color { 255, 40, 40, 255 });
    }

    // 6. Bottom Version Tag
    const std::string versionTag { "MOTION LAB 1996" };
    constexpr int kCharWidth { 4 };
    const int tagWidth { static_cast<int>(versionTag.length()) * kCharWidth };
    const int tagX { screenWidth - tagWidth - 24 };
    drawText(renderer, versionTag, tagX, screenHeight - 26, 1, dimmedColor);

    // 7. Pause Banner
    if (snapshot.isPaused) {
        const SDL_Rect pauseBox { screenWidth / 2 - 120, screenHeight / 2 - 40, 240, 80 };
        SDL_SetRenderDrawColor(renderer, 8, 9, 10, 220);
        SDL_RenderFillRect(renderer, &pauseBox);
        SDL_SetRenderDrawColor(renderer, 212, 171, 113, 255);
        SDL_RenderDrawRect(renderer, &pauseBox);
        drawText(renderer, "SIMULATION PAUSED", screenWidth / 2 - 100, screenHeight / 2 - 15, 2, goldColor);
        drawText(renderer, "PRESS P TO RESUME", screenWidth / 2 - 95, screenHeight / 2 + 10, 2, whiteColor);
    }
}

void TelemetryHud::render(
    SDL_Renderer* renderer, const TelemetrySnapshot& snapshot, int screenWidth, int screenHeight) noexcept
{
    const ExtendedTelemetrySnapshot defaultExtended { };
    render(renderer, snapshot, defaultExtended, screenWidth, screenHeight);
}

} // namespace motion_lab
