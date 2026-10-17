#pragma once

#include <algorithm>
#include <cmath>

namespace vehicle_sim::dynamics {

/// @struct TireThermalConfig
/// @brief Physical, thermodynamic, and material parameters for a tire thermal model.
struct TireThermalConfig {
    double surfaceThermalMass { 120.0 };   ///< Contact patch surface heat capacity in J/K.
    double bulkThermalMass { 1200.0 };     ///< Carcass bulk heat capacity in J/K.
    double cavityAirThermalMass { 180.0 };  ///< Inflation air heat capacity in J/K.

    double internalConductance { 45.0 };   ///< Surface-to-carcass heat conductance in W/K.
    double roadConductance { 60.0 };       ///< Contact patch to road surface conductance in W/K.
    double cavityConductance { 25.0 };     ///< Carcass-to-cavity air conductance in W/K.

    double baseConvectionCoeff { 15.0 };   ///< Stationary convective cooling coefficient in W/(m^2*K).
    double convectionSpeedScale { 2.4 };   ///< Speed-scaled convective exponent multiplier.

    double hysteresisCoeff { 0.012 };      ///< Carcass cyclic deflection loss factor C_hyst.
    double thermalConversionEff { 0.95 };  ///< Mechanical scrub to thermal energy efficiency.

    double coldTempMidpoint { 55.0 };      ///< Cold transition midpoint in Celsius.
    double coldSlopeWidth { 6.0 };         ///< Cold transition slope width in Celsius.
    double coldGripDeficit { 0.20 };       ///< Maximum grip reduction under cold conditions.

    double optimalTempMin { 75.0 };        ///< Lower threshold of peak grip window in Celsius.
    double optimalTempMax { 95.0 };        ///< Upper threshold of peak grip window in Celsius.
    double hotDegradationRate { 25.0 };    ///< Overheat degradation slope factor in Celsius.
    double maxThermalDecay { 0.35 };       ///< Maximum friction degradation ratio when overheated.

    double baseInflationPressure { 2.0 };  ///< Cold inflation pressure P_cold in bar.
    double referenceColdTemp { 25.0 };     ///< Reference cold ambient temperature in Celsius.
    double smokeThresholdTemp { 115.0 };   ///< Surface temperature triggering vapor smoke in Celsius.
};

/// @struct TireThermalState
/// @brief Instantaneous thermodynamic state of a single tire.
struct TireThermalState {
    double surfaceTemp { 25.0 };           ///< Contact patch surface temperature in Celsius.
    double carcassTemp { 25.0 };           ///< Bulk carcass temperature in Celsius.
    double cavityTemp { 25.0 };            ///< Inflation air cavity temperature in Celsius.
    double inflationPressure { 2.0 };      ///< Current pneumatic inflation pressure in bar.
    double effectiveMuMultiplier { 1.0 };  ///< Dynamic friction modifier mu_eff(T).
    double frictionPower { 0.0 };          ///< Instantaneous scrub friction power in Watts.
    double hysteresisPower { 0.0 };        ///< Instantaneous carcass flexing power in Watts.
    bool isOverheated { false };           ///< True if surface exceeds optimal grip plateau.
    bool isSmoking { false };              ///< True if generating tire vapor smoke.
};

/// @class TireThermalModel
/// @brief Industrial 3-node lumped capacitance tire thermodynamic evaluator.
/// @details Evaluates contact patch frictional heating, carcass rolling hysteresis,
/// airspeed-dependent convective cooling, road conduction, and non-linear grip drop.
class TireThermalModel {
public:
    /// @brief Default constructor initializing standard radial configuration.
    constexpr TireThermalModel() noexcept = default;

    /// @brief Parameterized constructor using custom thermal configuration.
    /// @param[in] config Thermal capacity, dissipation, and grip parameters.
    explicit TireThermalModel(const TireThermalConfig& config) noexcept;

    /// @brief Resets all temperatures to specified ambient temperature.
    /// @param[in] ambientTemp Equilibrium ambient temperature in Celsius.
    void reset(double ambientTemp = 25.0) noexcept;

    /// @brief Advances thermodynamic state across simulation timestep dt.
    /// @param[in] tractiveForce Longitudinal force Fx in Newtons.
    /// @param[in] corneringForce Lateral force Fy in Newtons.
    /// @param[in] normalLoad Vertical normal reaction Fz in Newtons.
    /// @param[in] slipVelLong Longitudinal sliding scrub velocity in m/s.
    /// @param[in] slipVelLat Lateral sliding scrub velocity in m/s.
    /// @param[in] wheelAngularSpeed Wheel rotational velocity omega in rad/s.
    /// @param[in] wheelRadius Effective rolling radius in meters.
    /// @param[in] relativeAirSpeed Relative chassis airspeed in m/s.
    /// @param[in] ambientTemp Ambient air temperature in Celsius.
    /// @param[in] roadTemp Asphalt road surface temperature in Celsius.
    /// @param[in] dt Physics timestep duration in seconds.
    void update(double tractiveForce,
                double corneringForce,
                double normalLoad,
                double slipVelLong,
                double slipVelLat,
                double wheelAngularSpeed,
                double wheelRadius,
                double relativeAirSpeed,
                double ambientTemp,
                double roadTemp,
                double dt) noexcept;

    /// @brief Evaluates dynamic friction multiplier for a given surface temperature.
    /// @param[in] surfaceTemp Target surface temperature in Celsius.
    /// @return Non-linear friction factor in range [1.0 - maxDecay, 1.0].
    [[nodiscard]] double evaluateGripFactor(double surfaceTemp) const noexcept;

    /// @brief Queries current surface temperature.
    /// @return Surface temperature in Celsius.
    [[nodiscard]] double getSurfaceTemp() const noexcept;

    /// @brief Queries current carcass temperature.
    /// @return Carcass bulk temperature in Celsius.
    [[nodiscard]] double getCarcassTemp() const noexcept;

    /// @brief Queries current inflation cavity pressure.
    /// @return Cavity pressure in bar.
    [[nodiscard]] double getCavityPressure() const noexcept;

    /// @brief Queries dynamic effective friction multiplier.
    /// @return Effective friction coefficient multiplier.
    [[nodiscard]] double getGripMultiplier() const noexcept;

    /// @brief Queries active thermal state snapshot.
    /// @return Const reference to TireThermalState.
    [[nodiscard]] const TireThermalState& getState() const noexcept;

    /// @brief Queries thermal configuration.
    /// @return Const reference to TireThermalConfig.
    [[nodiscard]] const TireThermalConfig& getConfig() const noexcept;

    /// @brief Reconfigures thermal parameters.
    /// @param[in] config New configuration structure.
    void setConfig(const TireThermalConfig& config) noexcept;

private:
    TireThermalConfig mConfig {};
    TireThermalState mState {};

    [[nodiscard]] double computeConvection(double airSpeed) const noexcept;
    void updatePressure() noexcept;
};

} // namespace vehicle_sim::dynamics
