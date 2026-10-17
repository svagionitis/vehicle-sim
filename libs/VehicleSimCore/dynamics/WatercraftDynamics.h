#pragma once

#include <cmath>

#include "libs/VehicleSimCore/dynamics/IVehicleDynamics.h"
#include "libs/VehicleSimCore/math/Vec2.h"

namespace vehicle_sim::physics {
class RigidBody2D;
} // namespace vehicle_sim::physics

namespace vehicle_sim::terrain {
class WaveGrid2D;
} // namespace vehicle_sim::terrain

namespace vehicle_sim::dynamics {

/// @struct WatercraftConfig
/// @brief Configuration parameters for 2D watercraft hydrodynamics.
/// @details Defines displacement and planing hull properties, Archimedes buoyancy,
/// waterline level, propeller capacity, water rudder turning authority, and heave/pitch damping.
struct WatercraftConfig {
    double mass { 1500.0 }; ///< Hull and payload structural mass in kg.
    double length { 5.5 }; ///< Hull overall length in meters.
    double beam { 2.2 }; ///< Hull beam width in meters.
    double draft { 0.6 }; ///< Submerged hull depth at rest in meters.
    double waterlineY { 0.0 }; ///< World space vertical Y level of water surface in meters.
    double waterDensity { 1000.0 }; ///< Water fluid density in kg/m^3.

    double maxPropellerThrust { 8500.0 }; ///< Peak propeller propulsion force in Newtons.
    double maxRudderAngle { 0.610865 }; ///< Maximum rudder deflection angle in radians (~35 deg).
    double rudderSlewRate { 3.5 }; ///< Water rudder steering slew rate in rad/s.
    math::Vec2 rudderOffset { -2.5, 0.0 }; ///< Local mount position of rudder and propeller in meters.

    double planingThresholdSpeed { 6.0 }; ///< Speed threshold where hydrodynamic lift initiates planing in m/s.
    double maxPlaningLift { 7000.0 }; ///< Maximum hydrodynamic planing lift force in Newtons.
    double displacementDragCoeff { 65.0 }; ///< Form drag coefficient in displacement mode in N/(m/s)^2.
    double planingDragCoeff { 18.0 }; ///< Reduced form drag coefficient in planing mode in N/(m/s)^2.

    double lateralResistance { 18000.0 }; ///< Keel and chine lateral slip resistance stiffness in N*s/m.
    double heaveDamping { 3500.0 }; ///< Vertical water oscillation damping coefficient in N*s/m.
    double pitchDamping { 2200.0 }; ///< Metacentric pitch attitude damping moment in N*m*s.
    double metacentricStiffness { 12000.0 }; ///< Hydrostatic pitch restoring moment stiffness in N*m/rad.
    math::Vec2 gravity { 0.0, -9.80665 }; ///< Ambient gravity acceleration vector in m/s^2.
};

/// @struct WatercraftInput
/// @brief Helmsman control commands for watercraft hydrodynamics.
struct WatercraftInput {
    double throttle { 0.0 }; ///< Commanded propeller throttle in [-1.0, 1.0].
    double rudder { 0.0 }; ///< Commanded water rudder steering deflection in [-1.0, 1.0].
};

/// @struct WatercraftTelemetry
/// @brief Diagnostic snapshot of internal watercraft hydrodynamics states.
struct WatercraftTelemetry {
    double speed { 0.0 }; ///< Hull forward speed through water in m/s.
    double draftDepth { 0.0 }; ///< Current submerged hull depth below water surface in meters.
    double buoyancyForce { 0.0 }; ///< Current Archimedes hydrostatic buoyancy force in Newtons.
    double planingLift { 0.0 }; ///< Current hydrodynamic planing lift force in Newtons.
    double dragForce { 0.0 }; ///< Total hydrodynamic water drag force in Newtons.
    double thrustForce { 0.0 }; ///< Current propeller propulsion force in Newtons.
    double rudderAngle { 0.0 }; ///< Current active water rudder steering angle in radians.
    bool isPlaning { false }; ///< True if hull has transitioned into high-speed planing regime.
    bool isSubmerged { true }; ///< True if hull is in contact with water body.
    double forwardVelocity { 0.0 }; ///< Longitudinal velocity along hull heading in m/s.
    double lateralVelocity { 0.0 }; ///< Transverse velocity across hull beam in m/s.
};

/// @class WatercraftDynamics
/// @brief 2D watercraft hydrodynamics coordinator.
/// @details Models Archimedes hydrostatic buoyancy, displacement-to-planing hull transitions,
/// propeller thrust vectoring via water rudder, keel lateral resistance, and metacentric stability.
class WatercraftDynamics final : public IVehicleDynamics {
public:
    /// @brief Default constructor using baseline watercraft configuration.
    WatercraftDynamics() noexcept;

    /// @brief Parameterized constructor with custom watercraft configuration.
    /// @param[in] config Watercraft hydrodynamics configuration parameters.
    explicit WatercraftDynamics(const WatercraftConfig& config) noexcept;

    /// @brief Advances watercraft simulation step by timestep dt.
    /// @details Computes submerged draft, buoyancy, planing lift, propeller thrust,
    /// water rudder moments, and applies accumulated forces to target rigid body.
    /// @param[in,out] body Target rigid body receiving forces and moments.
    /// @param[in] dt Timestep duration in seconds.
    void step(physics::RigidBody2D& body, double dt) noexcept override;

    /// @brief Resets transient control inputs and rudder deflection.
    void reset() noexcept override;

    /// @brief Queries whether the watercraft is airborne (e.g. jumped off a wave).
    /// @return True if hull is completely above the waterline.
    [[nodiscard]] bool isAirborne() const noexcept override;

    /// @brief Gets active watercraft configuration.
    /// @return Const reference to active WatercraftConfig.
    [[nodiscard]] const WatercraftConfig& getConfig() const noexcept;

    /// @brief Updates active watercraft configuration.
    /// @param[in] config New configuration parameters.
    void setConfig(const WatercraftConfig& config) noexcept;

    /// @brief Gets current helmsman control inputs.
    /// @return Const reference to active WatercraftInput.
    [[nodiscard]] const WatercraftInput& getInput() const noexcept;

    /// @brief Sets helmsman control inputs.
    /// @param[in] input New helmsman control commands.
    void setInput(const WatercraftInput& input) noexcept;

    /// @brief Gets diagnostic hydrodynamics telemetry snapshot.
    /// @return Const reference to WatercraftTelemetry.
    [[nodiscard]] const WatercraftTelemetry& getTelemetry() const noexcept;

    /// @brief Queries whether the hull is actively planing atop water.
    /// @return True if in high-speed planing regime.
    [[nodiscard]] bool isPlaning() const noexcept;

    /// @brief Queries whether hull is in contact with water.
    /// @return True if hull is partially or fully submerged.
    [[nodiscard]] bool isSubmerged() const noexcept;

    /// @brief Gets hull forward longitudinal speed.
    /// @return Speed in m/s.
    [[nodiscard]] double getSpeed() const noexcept;

    /// @brief Gets active rudder steering angle.
    /// @return Rudder angle in radians.
    [[nodiscard]] double getRudderAngle() const noexcept;

    /// @brief Attaches a 2D shallow water wave basin for dynamic wave interaction.
    /// @param[in] waveGrid Pointer to WaveGrid2D, or nullptr to disable.
    void attachWaveGrid(terrain::WaveGrid2D* waveGrid) noexcept;

    /// @brief Gets pointer to attached 2D wave basin.
    /// @return Pointer to active WaveGrid2D or nullptr.
    [[nodiscard]] terrain::WaveGrid2D* getWaveGrid() const noexcept;

private:
    WatercraftConfig mConfig { };
    WatercraftInput mInput { };
    WatercraftTelemetry mTelemetry { };
    double mRudderAngle { 0.0 };
    terrain::WaveGrid2D* mWaveGrid { nullptr };
};

} // namespace vehicle_sim::dynamics
