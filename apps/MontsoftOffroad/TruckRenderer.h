#pragma once

#include "apps/MontsoftOffroad/Camera2D.h"
#include "apps/MontsoftOffroad/CgaPaletteRenderer.h"
#include "libs/VehicleSimCore/dynamics/OffroadTruckDynamics.h"
#include "libs/VehicleSimCore/physics/RigidBody2D.h"

namespace montsoft_offroad {

/// @class TruckRenderer
/// @brief Articulated procedural 4WD desert truck rasterizer for CGA display.
/// @details Projects and renders the player truck with rotating chassis pitch,
/// independent telescopic coil-over springs, wishbone linkages, knobby tires with rotating
/// lugs, cabin glass with pilot silhouette, roll-cage, and dual weapon pods.
class TruckRenderer final {
public:
    /// @brief Default constructor.
    TruckRenderer() noexcept = default;

    /// @brief Destructor.
    ~TruckRenderer() noexcept = default;

    /// @brief Renders the articulated 4WD desert racing truck into the CGA framebuffer.
    /// @param[in,out] renderer Target CGA virtual framebuffer.
    /// @param[in] camera Active camera tracker performing coordinate transformation.
    /// @param[in] chassis Rigid body state of the vehicle chassis (position, rotation).
    /// @param[in] dynamics Vehicle dynamics coordinator providing axle telemetry and inputs.
    void render(
        CgaPaletteRenderer& renderer,
        const Camera2D& camera,
        const vehicle_sim::physics::RigidBody2D& chassis,
        const vehicle_sim::dynamics::OffroadTruckDynamics& dynamics
    ) noexcept;

    /// @brief Updates continuous wheel rotation angle for tread animation.
    /// @param[in] forwardVelocity Forward vehicle speed in m/s.
    /// @param[in] dt Frame delta time in seconds.
    void updateAnimation(double forwardVelocity, double dt) noexcept;

    /// @brief Resets wheel animation phase.
    void reset() noexcept;

private:
    void renderSuspensionStrut(
        CgaPaletteRenderer& renderer,
        const Camera2D& camera,
        const vehicle_sim::math::Vec2& mountWorldPos,
        const vehicle_sim::math::Vec2& hubWorldPos
    ) const noexcept;

    void renderKnobbyTire(
        CgaPaletteRenderer& renderer,
        const Camera2D& camera,
        const vehicle_sim::dynamics::OffroadAxleState& axleState,
        double tireRadius
    ) const noexcept;

    void renderChassisBody(
        CgaPaletteRenderer& renderer,
        const Camera2D& camera,
        const vehicle_sim::physics::RigidBody2D& chassis,
        bool jumpThrusterActive
    ) const noexcept;

    void renderWeapons(
        CgaPaletteRenderer& renderer,
        const Camera2D& camera,
        const vehicle_sim::physics::RigidBody2D& chassis
    ) const noexcept;

    double wheelRotationAngle_ { 0.0 };
};

} // namespace montsoft_offroad
