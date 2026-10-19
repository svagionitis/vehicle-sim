#include "apps/MontsoftOffroad/TruckRenderer.h"

#include <algorithm>
#include <cmath>

namespace montsoft_offroad {

using vehicle_sim::math::Vec2;

void TruckRenderer::updateAnimation(double forwardVelocity, double dt) noexcept
{
    constexpr double kNominalTireRadius { 0.42 };
    wheelRotationAngle_ += (forwardVelocity / kNominalTireRadius) * dt;

    constexpr double kTwoPi { 6.28318530717958647692 };
    if (wheelRotationAngle_ > kTwoPi) {
        wheelRotationAngle_ = std::fmod(wheelRotationAngle_, kTwoPi);
    } else if (wheelRotationAngle_ < -kTwoPi) {
        wheelRotationAngle_ = std::fmod(wheelRotationAngle_, kTwoPi);
    }
}

void TruckRenderer::reset() noexcept
{
    wheelRotationAngle_ = 0.0;
}

void TruckRenderer::render(
    CgaPaletteRenderer& renderer,
    const Camera2D& camera,
    const vehicle_sim::physics::RigidBody2D& chassis,
    const vehicle_sim::dynamics::OffroadTruckDynamics& dynamics
) noexcept
{
    const Vec2 com { chassis.getPosition() };
    const double theta { chassis.getOrientation() };
    const double cosT { std::cos(theta) };
    const double sinT { std::sin(theta) };

    const auto transformLocal = [&](double lx, double ly) noexcept -> Vec2 {
        return com + Vec2{ lx * cosT - ly * sinT, lx * sinT + ly * cosT };
    };

    // 1. Suspension Struts and Articulated Wishbones (Layer behind body)
    constexpr double kStrutMountY { -0.10 };
    const double rearMountX { dynamics.getConfig().axles[0].longitudinalOffset };
    const double frontMountX { dynamics.getConfig().axles[1].longitudinalOffset };

    const Vec2 rearMountWorld { transformLocal(rearMountX, kStrutMountY) };
    const Vec2 frontMountWorld { transformLocal(frontMountX, kStrutMountY) };

    renderSuspensionStrut(renderer, camera, rearMountWorld, dynamics.getAxleState(0).worldPos);
    renderSuspensionStrut(renderer, camera, frontMountWorld, dynamics.getAxleState(1).worldPos);

    // 2. Knobby Tires with Articulated Wheel Hubs
    renderKnobbyTire(renderer, camera, dynamics.getAxleState(0), dynamics.getConfig().axles[0].tireRadius);
    renderKnobbyTire(renderer, camera, dynamics.getAxleState(1), dynamics.getConfig().axles[1].tireRadius);

    // 3. Chassis Bodywork, Rollbar, and Cabin
    const bool jumpActive { dynamics.getInput().jump };
    renderChassisBody(renderer, camera, chassis, jumpActive);

    // 4. Weapons (Forward Rocket Silo and 65-deg Anti-Air Mortar)
    renderWeapons(renderer, camera, chassis);
}

void TruckRenderer::renderSuspensionStrut(
    CgaPaletteRenderer& renderer,
    const Camera2D& camera,
    const Vec2& mountWorldPos,
    const Vec2& hubWorldPos
) const noexcept
{
    const Vec2 screenMount { camera.worldToScreen(mountWorldPos) };
    const Vec2 screenHub { camera.worldToScreen(hubWorldPos) };

    const int mx { static_cast<int>(std::round(screenMount.x)) };
    const int my { static_cast<int>(std::round(screenMount.y)) };
    const int hx { static_cast<int>(std::round(screenHub.x)) };
    const int hy { static_cast<int>(std::round(screenHub.y)) };

    // Off-screen culling
    if (std::max(mx, hx) < -10 || std::min(mx, hx) > CgaPaletteRenderer::kNativeWidth + 10 ||
        std::max(my, hy) < -10 || std::min(my, hy) > CgaPaletteRenderer::kNativeHeight + 10) {
        return;
    }

    const ColorRgba strutColor{ 255, 255, 255, 255 }; // White telescopic shock
    const ColorRgba springColor{ 85, 255, 255, 255 }; // Cyan coil spring

    // Telescopic center damper rod
    renderer.drawLine(mx, my, hx, hy, strutColor);

    // Coil spring zig-zag turns
    const double dx { static_cast<double>(hx - mx) };
    const double dy { static_cast<double>(hy - my) };
    const double len { std::sqrt(dx * dx + dy * dy) };

    if (len > 4.0) {
        const double nx { -dy / len };
        const double ny { dx / len };
        constexpr std::size_t kCoils { 4U };
        constexpr double kCoilAmplitude { 3.0 };

        int prevX { mx };
        int prevY { my };

        for (std::size_t i { 1 }; i <= kCoils * 2U; ++i) {
            const double t { static_cast<double>(i) / static_cast<double>(kCoils * 2U + 1U) };
            const double side { (i % 2U == 0U) ? 1.0 : -1.0 };
            const int cx { static_cast<int>(std::round(mx + dx * t + nx * side * kCoilAmplitude)) };
            const int cy { static_cast<int>(std::round(my + dy * t + ny * side * kCoilAmplitude)) };

            renderer.drawLine(prevX, prevY, cx, cy, springColor);
            prevX = cx;
            prevY = cy;
        }
        renderer.drawLine(prevX, prevY, hx, hy, springColor);
    }
}

void TruckRenderer::renderKnobbyTire(
    CgaPaletteRenderer& renderer,
    const Camera2D& camera,
    const vehicle_sim::dynamics::OffroadAxleState& axleState,
    double tireRadius
) const noexcept
{
    const Vec2 screenHub { camera.worldToScreen(axleState.worldPos) };
    const int hx { static_cast<int>(std::round(screenHub.x)) };
    const int hy { static_cast<int>(std::round(screenHub.y)) };

    const int radiusPx { std::max(4, static_cast<int>(std::round(tireRadius * camera.getConfig().pixelsPerMeter))) };

    if (hx < -radiusPx || hx >= CgaPaletteRenderer::kNativeWidth + radiusPx ||
        hy < -radiusPx || hy >= CgaPaletteRenderer::kNativeHeight + radiusPx) {
        return;
    }

    const ColorRgba tireRubber{ 0, 0, 0, 255 };       // Black rubber
    const ColorRgba treadColor{ 255, 255, 255, 255 };  // White outer tread/lugs
    const ColorRgba rimColor{ 85, 255, 255, 255 };     // Cyan hub & wheel rim

    // Solid rubber core
    renderer.fillCircle(hx, hy, radiusPx, tireRubber);
    renderer.drawCircle(hx, hy, radiusPx, treadColor);

    // Inner rim
    const int rimRadius { std::max(2, radiusPx - 3) };
    renderer.drawCircle(hx, hy, rimRadius, rimColor);

    // Radial knobby tread lugs
    constexpr std::size_t kLugCount { 6U };
    constexpr double kTwoPi { 6.28318530717958647692 };
    for (std::size_t i { 0 }; i < kLugCount; ++i) {
        const double angle { wheelRotationAngle_ + static_cast<double>(i) * (kTwoPi / static_cast<double>(kLugCount)) };
        const int lx { static_cast<int>(std::round(hx + (radiusPx + 1) * std::cos(angle))) };
        const int ly { static_cast<int>(std::round(hy - (radiusPx + 1) * std::sin(angle))) };
        renderer.setPixel(lx, ly, treadColor);
    }

    // Center hubcap nut
    renderer.fillRect(hx - 1, hy - 1, 3, 3, rimColor);
    renderer.setPixel(hx, hy, treadColor);
}

void TruckRenderer::renderChassisBody(
    CgaPaletteRenderer& renderer,
    const Camera2D& camera,
    const vehicle_sim::physics::RigidBody2D& chassis,
    bool jumpThrusterActive
) const noexcept
{
    const Vec2 com { chassis.getPosition() };
    const double theta { chassis.getOrientation() };
    const double cosT { std::cos(theta) };
    const double sinT { std::sin(theta) };

    const auto toScreen = [&](double lx, double ly) noexcept -> Vec2 {
        const Vec2 worldPos { com + Vec2{ lx * cosT - ly * sinT, lx * sinT + ly * cosT } };
        return camera.worldToScreen(worldPos);
    };

    const ColorRgba bodyMagenta{ 255, 85, 255, 255 }; // High-intensity CGA Magenta
    const ColorRgba highlightWhite{ 255, 255, 255, 255 };
    const ColorRgba glassCyan{ 85, 255, 255, 255 };
    const ColorRgba darkBlack{ 0, 0, 0, 255 };
    const ColorRgba sparkYellow{ 255, 255, 85, 255 };

    // 1. Lower Chassis Skidplate & Undercarriage Frame
    const Vec2 pBedRear { toScreen(-1.60, -0.05) };
    const Vec2 pBumperFront { toScreen(1.65, -0.05) };

    const Vec2 pBedTopRear { toScreen(-1.60, 0.35) };
    const Vec2 pBedTopFront { toScreen(-0.30, 0.35) };
    const Vec2 pCabRoofRear { toScreen(-0.30, 0.75) };
    const Vec2 pCabRoofFront { toScreen(0.40, 0.75) };
    const Vec2 pHoodFront { toScreen(1.55, 0.30) };

    // Main bodywork silhouette
    renderer.drawLine(static_cast<int>(pBedRear.x), static_cast<int>(pBedRear.y),
                      static_cast<int>(pBedTopRear.x), static_cast<int>(pBedTopRear.y), highlightWhite);
    renderer.drawLine(static_cast<int>(pBedTopRear.x), static_cast<int>(pBedTopRear.y),
                      static_cast<int>(pBedTopFront.x), static_cast<int>(pBedTopFront.y), bodyMagenta);
    renderer.drawLine(static_cast<int>(pBedTopFront.x), static_cast<int>(pBedTopFront.y),
                      static_cast<int>(pCabRoofRear.x), static_cast<int>(pCabRoofRear.y), highlightWhite);
    renderer.drawLine(static_cast<int>(pCabRoofRear.x), static_cast<int>(pCabRoofRear.y),
                      static_cast<int>(pCabRoofFront.x), static_cast<int>(pCabRoofFront.y), highlightWhite);
    renderer.drawLine(static_cast<int>(pCabRoofFront.x), static_cast<int>(pCabRoofFront.y),
                      static_cast<int>(pHoodFront.x), static_cast<int>(pHoodFront.y), highlightWhite); // Windshield
    renderer.drawLine(static_cast<int>(pHoodFront.x), static_cast<int>(pHoodFront.y),
                      static_cast<int>(pBumperFront.x), static_cast<int>(pBumperFront.y), highlightWhite);
    renderer.drawLine(static_cast<int>(pBumperFront.x), static_cast<int>(pBumperFront.y),
                      static_cast<int>(pBedRear.x), static_cast<int>(pBedRear.y), bodyMagenta);

    // Fill inner bodywork with Magenta paneling
    const double ppm { camera.getConfig().pixelsPerMeter };
    const double scale { ppm / 20.0 };

    const Vec2 pCenter { toScreen(0.0, 0.15) };
    const int cx { static_cast<int>(pCenter.x) };
    const int cy { static_cast<int>(pCenter.y) };
    const int bodyW { std::max(10, static_cast<int>(std::round(36.0 * scale))) };
    const int bodyH { std::max(4, static_cast<int>(std::round(8.0 * scale))) };
    renderer.fillRect(cx - bodyW / 2, cy - bodyH / 2, bodyW, bodyH, bodyMagenta);

    // 2. Cockpit Windshield Glass (Cyan) & Pilot Silhouette (Black)
    const Vec2 pGlassTop { toScreen(0.35, 0.70) };
    const int gx { static_cast<int>(pGlassTop.x) };
    const int gy { static_cast<int>(pGlassTop.y) };
    const int gw { std::max(4, static_cast<int>(std::round(10.0 * scale))) };
    const int gh { std::max(3, static_cast<int>(std::round(6.0 * scale))) };
    renderer.fillRect(gx - 3, gy + 1, gw, gh, glassCyan);
    // Pilot silhouette
    const int pw { std::max(2, static_cast<int>(std::round(4.0 * scale))) };
    const int ph { std::max(2, static_cast<int>(std::round(4.0 * scale))) };
    renderer.fillRect(gx, gy + 2, pw, ph, darkBlack);

    // 3. Rollbar & Roll-cage Truss
    const Vec2 pRollbarTop { toScreen(-0.25, 0.75) };
    const Vec2 pRollbarBrace { toScreen(-1.10, 0.35) };
    renderer.drawLine(static_cast<int>(pBedTopFront.x), static_cast<int>(pBedTopFront.y),
                      static_cast<int>(pRollbarTop.x), static_cast<int>(pRollbarTop.y), highlightWhite);
    renderer.drawLine(static_cast<int>(pRollbarTop.x), static_cast<int>(pRollbarTop.y),
                      static_cast<int>(pRollbarBrace.x), static_cast<int>(pRollbarBrace.y), highlightWhite);

    // 4. Spare Tire Mounted in Bed
    const Vec2 pSpare { toScreen(-0.85, 0.35) };
    const int spareR { std::max(2, static_cast<int>(std::round(4.0 * scale))) };
    renderer.fillCircle(static_cast<int>(pSpare.x), static_cast<int>(pSpare.y), spareR, darkBlack);
    renderer.drawCircle(static_cast<int>(pSpare.x), static_cast<int>(pSpare.y), spareR, highlightWhite);

    // 5. Front Tubular Bull-Bar Bumper & Spotlights
    const Vec2 pBumperUpper { toScreen(1.75, 0.25) };
    const Vec2 pBumperLower { toScreen(1.75, -0.15) };
    renderer.drawLine(static_cast<int>(pBumperUpper.x), static_cast<int>(pBumperUpper.y),
                      static_cast<int>(pBumperLower.x), static_cast<int>(pBumperLower.y), highlightWhite);
    const int spotR { std::max(1, static_cast<int>(std::round(2.0 * scale))) };
    renderer.drawCircle(static_cast<int>(pBumperUpper.x), static_cast<int>(pBumperUpper.y), spotR, sparkYellow);

    // 6. Rear Whip Antenna with Flag
    const Vec2 pAntennaBase { toScreen(-1.55, 0.35) };
    const Vec2 pAntennaTip { toScreen(-1.75, 0.95) };
    renderer.drawLine(static_cast<int>(pAntennaBase.x), static_cast<int>(pAntennaBase.y),
                      static_cast<int>(pAntennaTip.x), static_cast<int>(pAntennaTip.y), highlightWhite);
    const int flagW { std::max(2, static_cast<int>(std::round(4.0 * scale))) };
    const int flagH { std::max(2, static_cast<int>(std::round(3.0 * scale))) };
    renderer.fillRect(static_cast<int>(pAntennaTip.x), static_cast<int>(pAntennaTip.y), flagW, flagH, sparkYellow);

    // 7. Undercarriage Jump Thruster Nozzle
    const Vec2 pThruster { toScreen(0.0, -0.12) };
    const int tx { static_cast<int>(pThruster.x) };
    const int ty { static_cast<int>(pThruster.y) };
    const int thW { std::max(3, static_cast<int>(std::round(6.0 * scale))) };
    const int thH { std::max(1, static_cast<int>(std::round(2.0 * scale))) };
    renderer.fillRect(tx - thW / 2, ty, thW, thH, highlightWhite);
    if (jumpThrusterActive) {
        const int flameW { std::max(4, static_cast<int>(std::round(8.0 * scale))) };
        const int flameH { std::max(2, static_cast<int>(std::round(3.0 * scale))) };
        renderer.fillRect(tx - flameW / 2, ty + thH, flameW, flameH, sparkYellow);
        const int coreW { std::max(2, static_cast<int>(std::round(4.0 * scale))) };
        renderer.fillRect(tx - coreW / 2, ty + thH + flameH, coreW, 2, highlightWhite);
    }
}

void TruckRenderer::renderWeapons(
    CgaPaletteRenderer& renderer,
    const Camera2D& camera,
    const vehicle_sim::physics::RigidBody2D& chassis
) const noexcept
{
    const Vec2 com { chassis.getPosition() };
    const double theta { chassis.getOrientation() };
    const double cosT { std::cos(theta) };
    const double sinT { std::sin(theta) };

    const auto toScreen = [&](double lx, double ly) noexcept -> Vec2 {
        const Vec2 worldPos { com + Vec2{ lx * cosT - ly * sinT, lx * sinT + ly * cosT } };
        return camera.worldToScreen(worldPos);
    };

    const ColorRgba weaponWhite{ 255, 255, 255, 255 };
    const ColorRgba weaponCyan{ 85, 255, 255, 255 };
    const double ppm { camera.getConfig().pixelsPerMeter };
    const double scale { ppm / 20.0 };

    // 1. Forward Heavy Rocket Launch Tube (Horizontal along hood)
    const Vec2 pRocketBase { toScreen(0.85, 0.28) };
    const Vec2 pRocketMuzzle { toScreen(1.65, 0.28) };
    const int rbx { static_cast<int>(pRocketBase.x) };
    const int rby { static_cast<int>(pRocketBase.y) };
    const int rmx { static_cast<int>(pRocketMuzzle.x) };
    const int rmy { static_cast<int>(pRocketMuzzle.y) };

    renderer.drawLine(rbx, rby, rmx, rmy, weaponWhite);
    renderer.drawLine(rbx, rby - 1, rmx, rmy - 1, weaponCyan);
    const int muzzW { std::max(2, static_cast<int>(std::round(3.0 * scale))) };
    const int muzzH { std::max(2, static_cast<int>(std::round(4.0 * scale))) };
    renderer.fillRect(rmx - 1, rmy - muzzH / 2, muzzW, muzzH, weaponWhite); // Muzzle bell

    // 2. High-Angle Anti-Air Mortar Tube (65 degrees elevation behind cabin)
    constexpr double kMortarAngle { 1.134464 }; // ~65 deg in radians
    constexpr double kMortarLen { 0.55 };
    const double mlx { -0.35 + kMortarLen * std::cos(kMortarAngle) };
    const double mly { 0.45 + kMortarLen * std::sin(kMortarAngle) };

    const Vec2 pMortarBase { toScreen(-0.35, 0.45) };
    const Vec2 pMortarMuzzle { toScreen(mlx, mly) };
    const int mbx { static_cast<int>(pMortarBase.x) };
    const int mby { static_cast<int>(pMortarBase.y) };
    const int mmx { static_cast<int>(pMortarMuzzle.x) };
    const int mmy { static_cast<int>(pMortarMuzzle.y) };

    renderer.drawLine(mbx, mby, mmx, mmy, weaponWhite);
    renderer.drawLine(mbx + 1, mby, mmx + 1, mmy, weaponCyan);
    const int mortMuzz { std::max(2, static_cast<int>(std::round(3.0 * scale))) };
    renderer.fillRect(mmx - 1, mmy - 1, mortMuzz, mortMuzz, weaponWhite); // Mortar muzzle
}

} // namespace montsoft_offroad
