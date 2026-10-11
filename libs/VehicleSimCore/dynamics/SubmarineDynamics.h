#pragma once

#include <cmath>

#include "libs/VehicleSimCore/dynamics/IVehicleDynamics.h"
#include "libs/VehicleSimCore/math/Vec2.h"

namespace vehicle_sim::physics {
class RigidBody2D;
} // namespace vehicle_sim::physics

namespace vehicle_sim::dynamics {

/// @struct SubmarineConfig
/// @brief Configuration parameters for 2D submarine underwater hydrodynamics.
/// @details Defines displacement hull volume, dry mass, ballast tank capacity,
/// pumping rates, propulsion thrust, hydrofoil dive plane deflection, and hydrodynamic drag.
struct SubmarineConfig {
    double dryMass { 45000.0 }; ///< Empty structural dry mass in kg.
    double hullVolume { 52.0 }; ///< Outer displacement hull volume in m^3.
    double waterDensity { 1025.0 }; ///< Ambient sea water density in kg/m^3.
    double waterlineY { 0.0 }; ///< Surface water level Y coordinate in meters.

    double maxBallastMass { 12000.0 }; ///< Maximum water capacity of ballast tanks in kg.
    double initialBallastMass { 7000.0 }; ///< Initial water ballast mass in kg.
    double pumpRate { 3000.0 }; ///< Water flood / compressed air blow rate in kg/s.

    double maxThrust { 48000.0 }; ///< Peak screw propeller propulsion force in Newtons.
    double maxRudderAngle { 0.523599 }; ///< Maximum rudder deflection angle in radians (~30 deg).
    double rudderSlewRate { 2.5 }; ///< Water rudder steering slew rate in rad/s.
    double rudderDistance { 10.0 }; ///< Distance from center of mass to stern rudder in meters.

    double maxDivePlaneAngle { 0.436332 }; ///< Maximum dive plane angle in radians (~25 deg).
    double divePlaneSlewRate { 2.0 }; ///< Hydrofoil dive plane slew rate in rad/s.
    double divePlaneDistance { 8.0 }; ///< Distance from center of mass to bow dive planes in meters.
    double divePlaneLiftCoeff { 1200.0 }; ///< Dive plane dynamic lift coefficient in N/(rad*(m/s)^2).

    double longitudinalDragCoeff { 45.0 }; ///< Streamlined longitudinal water drag in N/(m/s)^2.
    double lateralDragCoeff { 320.0 }; ///< Broadside/vertical water form drag in N/(m/s)^2.
    double rotationalDamping { 18000.0 }; ///< Angular viscous water damping moment in N*m*s.
    double metacentricStiffness { 45000.0 }; ///< Hydrostatic pitch restoring moment stiffness in N*m/rad.

    double maxOperatingDepth { 300.0 }; ///< Safe maximum operating depth in meters.
    double crushDepth { 450.0 }; ///< Critical hull crush depth in meters.
    math::Vec2 gravity { 0.0, -9.80665 }; ///< Ambient gravity acceleration vector in m/s^2.
};

/// @struct SubmarineInput
/// @brief Helmsman and ballast control commands for submarine hydrodynamics.
struct SubmarineInput {
    double throttle { 0.0 }; ///< Commanded propeller throttle in [-1.0, 1.0].
    double rudder { 0.0 }; ///< Commanded water rudder steering deflection in [-1.0, 1.0].
    double divePlane { 0.0 }; ///< Commanded hydrofoil dive plane deflection in [-1.0, 1.0].
    double floodRate { 0.0 }; ///< Commanded ballast tank flooding rate in [0.0, 1.0].
    double blowRate { 0.0 }; ///< Commanded high-pressure air tank blow rate in [0.0, 1.0].
};

/// @struct SubmarineTelemetry
/// @brief Diagnostic snapshot of internal submarine hydrodynamics states.
struct SubmarineTelemetry {
    double depth { 0.0 }; ///< Depth below waterline surface in meters (positive downwards).
    double ballastMass { 0.0 }; ///< Current water ballast mass in kg.
    double ballastRatio { 0.0 }; ///< Fraction of ballast capacity filled in [0.0, 1.0].
    double totalMass { 0.0 }; ///< Combined dry mass and ballast mass in kg.
    double netBuoyancy { 0.0 }; ///< Net vertical buoyancy force (Archimedes minus gravity) in Newtons.
    double thrustForce { 0.0 }; ///< Current propeller propulsion force in Newtons.
    double divePlaneAngle { 0.0 }; ///< Active dive plane hydrofoil deflection angle in radians.
    double rudderAngle { 0.0 }; ///< Active stern water rudder deflection angle in radians.
    double forwardSpeed { 0.0 }; ///< Longitudinal forward speed through water in m/s.
    double verticalSpeed { 0.0 }; ///< Vertical speed through water in m/s.
    bool isSubmerged { false }; ///< True if hull is below the waterline.
    bool isCrushed { false }; ///< True if submarine exceeded crush depth.
};

/// @class SubmarineDynamics
/// @brief 2D submarine underwater hydrodynamics coordinator.
/// @details Simulates Archimedes hydrostatic buoyancy, variable ballast flooding and blowing,
/// stern propeller thrust, hydrofoil dive planes for pitch and vertical control,
/// deep-water viscous drag, metacentric righting stability, and hull depth pressure.
class SubmarineDynamics final : public IVehicleDynamics {
public:
    /// @brief Default constructor using baseline submarine configuration.
    SubmarineDynamics() noexcept;

    /// @brief Parameterized constructor with custom submarine configuration.
    /// @param[in] config Submarine configuration parameters.
    explicit SubmarineDynamics(const SubmarineConfig& config) noexcept;

    /// @brief Advances submarine simulation step by timestep dt.
    /// @details Updates ballast mass, computes net buoyancy, propeller thrust,
    /// dive plane lift moments, water rudder forces, and applies forces to rigid body.
    /// @param[in,out] body Target rigid body receiving forces and moments.
    /// @param[in] dt Timestep duration in seconds.
    void step(physics::RigidBody2D& body, double dt) noexcept override;

    /// @brief Resets transient control inputs and deflections.
    void reset() noexcept override;

    /// @brief Queries whether the submarine is airborne (above the water surface).
    /// @return True if hull is fully elevated above waterline level.
    [[nodiscard]] bool isAirborne() const noexcept override;

    /// @brief Gets active submarine configuration.
    /// @return Const reference to active SubmarineConfig.
    [[nodiscard]] const SubmarineConfig& getConfig() const noexcept;

    /// @brief Updates active submarine configuration.
    /// @param[in] config New configuration parameters.
    void setConfig(const SubmarineConfig& config) noexcept;

    /// @brief Gets current helmsman control inputs.
    /// @return Const reference to active SubmarineInput.
    [[nodiscard]] const SubmarineInput& getInput() const noexcept;

    /// @brief Sets helmsman control inputs.
    /// @param[in] input New helmsman control commands.
    void setInput(const SubmarineInput& input) noexcept;

    /// @brief Gets diagnostic hydrodynamics telemetry snapshot.
    /// @return Const reference to SubmarineTelemetry.
    [[nodiscard]] const SubmarineTelemetry& getTelemetry() const noexcept;

    /// @brief Queries whether the submarine hull is submerged.
    /// @return True if hull is below the waterline.
    [[nodiscard]] bool isSubmerged() const noexcept;

    /// @brief Queries whether the submarine hull was crushed by hydrostatic pressure.
    /// @return True if depth exceeds crush depth.
    [[nodiscard]] bool isCrushed() const noexcept;

    /// @brief Gets current submerged depth in meters.
    /// @return Depth below surface in meters.
    [[nodiscard]] double getDepth() const noexcept;

    /// @brief Gets net vertical buoyancy force.
    /// @return Net buoyancy in Newtons.
    [[nodiscard]] double getNetBuoyancy() const noexcept;

    /// @brief Gets active water ballast mass.
    /// @return Ballast mass in kg.
    [[nodiscard]] double getBallastMass() const noexcept;

private:
    SubmarineConfig mConfig { };
    SubmarineInput mInput { };
    SubmarineTelemetry mTelemetry { };
    double mBallastMass { 0.0 };
    double mRudderAngle { 0.0 };
    double mDivePlaneAngle { 0.0 };
};

} // namespace vehicle_sim::dynamics
