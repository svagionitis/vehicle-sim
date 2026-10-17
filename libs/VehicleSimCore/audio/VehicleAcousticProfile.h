#pragma once

#include <cstdint>

namespace vehicle_sim::audio {

/// @enum EngineConfiguration
/// @brief Thermodynamic engine cylinder arrangement and firing symmetry.
enum class EngineConfiguration : std::uint8_t {
    Inline4,         ///< 4-cylinder even firing (180 deg).
    V6,              ///< 6-cylinder even firing (120 deg).
    CrossplaneV8,    ///< 8-cylinder uneven firing (90-180-270-90 deg American muscle burble).
    FlatplaneV8,     ///< 8-cylinder even firing (180 deg European supercar scream).
    V10,             ///< 10-cylinder high-rev even firing (72 deg Formula howl).
    HeavyDieselV12,  ///< 12-cylinder heavy industrial diesel with combustion knock.
    HeavyDieselI6,   ///< 6-cylinder heavy commercial turbo diesel.
    ElectricPWM,     ///< Electric inverter pulse-width modulation carrier.
    SciFiWarp        ///< Sci-fi dual-oscillator frequency modulation warp drive.
};

/// @enum ForcedInductionType
/// @brief Induction pressurization acoustic type.
enum class ForcedInductionType : std::uint8_t {
    None,            ///< Naturally aspirated.
    Turbocharger,    ///< Exhaust turbine spool whistle, blow-off valve & compressor surge.
    Supercharger     ///< Crank-geared positive displacement blower whine.
};

/// @struct VehicleAcousticProfile
/// @brief Complete acoustic tuning parameter set for a vehicle chassis.
struct VehicleAcousticProfile {
    EngineConfiguration engineConfig { EngineConfiguration::Inline4 };
    ForcedInductionType inductionType { ForcedInductionType::None };

    // Combustion parameters
    double idleFrequency { 45.0 };          ///< Base firing frequency at idle in Hz.
    double redlineFrequency { 320.0 };      ///< Firing frequency at redline in Hz.
    double cylinderHarmonicWeight { 0.65 }; ///< Relative weight of cylinder harmonics.
    double subHarmonicWeight { 0.35 };      ///< Relative weight of sub-harmonics.
    double dieselKnockIntensity { 0.0 };    ///< Transient combustion knock [0.0, 1.0].

    // Exhaust formant chamber parameters
    double exhaustResonanceFreq1 { 140.0 };  ///< Primary pipe cavity resonance in Hz.
    double exhaustResonanceFreq2 { 580.0 };  ///< Collector pipe resonance in Hz.

    // Forced induction parameters
    double turboSpoolRate { 0.45 };         ///< Turbo lag rise time constant in seconds.
    double turboWhistleMaxFreq { 5200.0 };  ///< Peak whistle frequency at full boost in Hz.
    double superchargerGearRatio { 2.40 };  ///< Pulley gear drive ratio for blower whine.

    // Exhaust overrun & backfire parameters
    bool hasOverrunPops { true };           ///< True to generate decel exhaust pops.
    double overrunPopIntensity { 0.60 };    ///< Decel pop density and amplitude [0.0, 1.0].

    // Drivetrain & auxiliary acoustics
    bool hasStraightCutGearbox { false };   ///< True to synthesize straight-cut dogbox whine.
    bool hasPneumaticAirBrakes { false };   ///< True to synthesize commercial truck air brakes.
    bool hasTrackCrawlClatter { false };    ///< True to synthesize metallic tank track clatter.
};

/// @class AcousticProfileRegistry
/// @brief Registry providing calibrated acoustic profiles for Motion Lab fleet.
class AcousticProfileRegistry {
public:
    /// @brief Retrieves the calibrated acoustic profile for a vehicle index.
    /// @param[in] vehicleIndex Vehicle identifier index in [0, 11].
    /// @return Reference to static calibrated VehicleAcousticProfile.
    [[nodiscard]] static const VehicleAcousticProfile& getProfile(int vehicleIndex) noexcept;
};

} // namespace vehicle_sim::audio
