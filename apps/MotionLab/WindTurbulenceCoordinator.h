#pragma once

#include "apps/MotionLab/MotionLabVehicleType.h"
#include "libs/VehicleSimCore/math/Vec2.h"
#include "libs/VehicleSimCore/physics/FluidDrag2D.h"
#include "libs/VehicleSimCore/physics/FluidField2D.h"
#include "libs/VehicleSimCore/physics/RigidBody2D.h"

namespace motion_lab {

/// @struct WindTelemetrySnapshot
/// @brief Telemetry snapshot of environmental wind and active vehicle aerodynamics.
struct WindTelemetrySnapshot {
    vehicle_sim::math::Vec2 windVelocity { 0.0, 0.0 };
    vehicle_sim::math::Vec2 gustPerturbation { 0.0, 0.0 };
    double trueWindSpeed { 0.0 };          ///< True wind speed in m/s.
    double trueWindHeadingDeg { 0.0 };     ///< Meteorological wind heading [0, 360) deg.
    double relativeAirspeed { 0.0 };       ///< Apparent relative airspeed in m/s.
    double relativeAirspeedMph { 0.0 };    ///< Apparent relative airspeed in mph.
    double apparentWindAngleDeg { 0.0 };   ///< Apparent wind angle relative to chassis bow in deg.
    double lateralAeroForce { 0.0 };       ///< Lateral crosswind force in Newtons.
    double yawOverturningMoment { 0.0 };   ///< Aerodynamic yaw moment in N*m.
    double downforceLoad { 0.0 };          ///< Aerodynamic downforce in Newtons.
    bool crosswindAlert { false };         ///< True if lateral aero force exceeds threshold.
    bool gustPulseActive { false };        ///< True if 1-cosine discrete gust is in progress.
    vehicle_sim::physics::WindEnvironmentPreset activePreset {
        vehicle_sim::physics::WindEnvironmentPreset::Calm
    };
};

/// @class WindTurbulenceCoordinator
/// @brief Coordinates environmental atmospheric wind, turbulence, and vehicle aerodynamic load injection.
class WindTurbulenceCoordinator {
public:
    WindTurbulenceCoordinator() noexcept;

    /// @brief Advances atmospheric simulation and injects aerodynamic forces into vehicle body.
    /// @param[in,out] body Target chassis rigid body.
    /// @param[in] vehicleType Active vehicle model category.
    /// @param[in] dt Fixed physics timestep in seconds.
    void stepPhysics(vehicle_sim::physics::RigidBody2D& body,
                     MotionLabVehicleType vehicleType,
                     double dt) noexcept;

    /// @brief Applies a named wind environment preset.
    /// @param[in] preset Target environmental preset enum.
    void setPreset(vehicle_sim::physics::WindEnvironmentPreset preset) noexcept;

    /// @brief Cycles sequentially to the next environmental wind preset.
    void cycleNextPreset() noexcept;

    /// @brief Triggers an isolated 1-Cosine discrete gust pulse.
    /// @param[in] peakVelocity Peak gust perturbation speed in m/s.
    /// @param[in] durationPulse Total gust duration in seconds.
    void triggerDiscreteGust(double peakVelocity, double durationPulse) noexcept;

    /// @brief Multiplies current wind speed by an adjustment scale factor.
    /// @param[in] scaleFactor Multiplier factor (e.g. 1.25 or 0.80).
    void adjustWindSpeed(double scaleFactor) noexcept;

    /// @brief Increments meteorological wind heading angle.
    /// @param[in] deltaDegrees Offset in degrees (e.g. +15 or -15).
    void rotateWindHeading(double deltaDegrees) noexcept;

    /// @brief Toggles wind simulation active state.
    void toggleWindActive() noexcept;

    /// @brief Sets wind simulation active state explicitly.
    /// @param[in] active True to enable aerodynamic loads.
    void setWindActive(bool active) noexcept;

    /// @brief Checks if dynamic wind simulation is active.
    [[nodiscard]] bool isWindActive() const noexcept;

    /// @brief Queries current environmental wind telemetry snapshot.
    [[nodiscard]] const WindTelemetrySnapshot& getSnapshot() const noexcept;

    /// @brief Samples local wind velocity at a specific world coordinate.
    /// @param[in] worldPos World coordinates in meters.
    /// @return Instantaneous wind velocity in m/s.
    [[nodiscard]] vehicle_sim::math::Vec2 sampleWindAt(
        const vehicle_sim::math::Vec2& worldPos) const noexcept;

    /// @brief Clears active gusts and resets to calm default.
    void reset() noexcept;

    /// @brief Retrieves the active aerodynamic properties for a vehicle model.
    /// @param[in] type Target vehicle type.
    /// @return FluidDragProperties struct.
    [[nodiscard]] vehicle_sim::physics::FluidDragProperties getAeroProps(
        MotionLabVehicleType type) const noexcept;

    /// @brief Retrieves the active downforce coefficient for a vehicle model.
    /// @param[in] type Target vehicle type.
    /// @return Downforce coefficient CL * Area (m^2).
    [[nodiscard]] double getDownforceCoeff(
        MotionLabVehicleType type) const noexcept;

private:
    void applyWindPreset(vehicle_sim::physics::WindEnvironmentPreset preset) noexcept;
    void updateWindVector() noexcept;

    vehicle_sim::physics::FluidField2D mFluidField { };
    WindTelemetrySnapshot mSnapshot { };
    double mSimulationTime { 0.0 };
    double mWindSpeed { 0.0 };
    double mWindHeadingDeg { 0.0 };
    double mGustTimer { 0.0 };
    double mGustDuration { 0.0 };
    bool mWindActive { true };
};

} // namespace motion_lab
