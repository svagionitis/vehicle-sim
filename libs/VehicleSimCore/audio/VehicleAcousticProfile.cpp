#include "libs/VehicleSimCore/audio/VehicleAcousticProfile.h"

#include <algorithm>
#include <array>

namespace vehicle_sim::audio {

namespace {

// Static table of 12 vehicle acoustic profiles
constexpr std::array<VehicleAcousticProfile, 12> kFleetProfiles {{
    // 0: Classic Car (F1) - Inline-4, naturally aspirated, mild overrun crackle
    {
        EngineConfiguration::Inline4,
        ForcedInductionType::None,
        55.0,   // idleFrequency
        260.0,  // redlineFrequency
        0.70,   // cylinderHarmonicWeight
        0.30,   // subHarmonicWeight
        0.00,   // dieselKnockIntensity
        150.0,  // exhaustResonanceFreq1
        520.0,  // exhaustResonanceFreq2
        0.40,   // turboSpoolRate
        4500.0, // turboWhistleMaxFreq
        2.20,   // superchargerGearRatio
        true,   // hasOverrunPops
        0.35,   // overrunPopIntensity
        false,  // hasStraightCutGearbox
        false,  // hasPneumaticAirBrakes
        false   // hasTrackCrawlClatter
    },
    // 1: Race Car (F2) - High-rev V10 (18,000 RPM scream), straight-cut dogbox, aggressive overrun
    {
        EngineConfiguration::V10,
        ForcedInductionType::None,
        90.0,   // idleFrequency
        580.0,  // redlineFrequency
        0.85,   // cylinderHarmonicWeight
        0.15,   // subHarmonicWeight
        0.00,   // dieselKnockIntensity
        180.0,  // exhaustResonanceFreq1
        740.0,  // exhaustResonanceFreq2
        0.30,   // turboSpoolRate
        6000.0, // turboWhistleMaxFreq
        2.50,   // superchargerGearRatio
        true,   // hasOverrunPops
        0.85,   // overrunPopIntensity
        true,   // hasStraightCutGearbox
        false,  // hasPneumaticAirBrakes
        false   // hasTrackCrawlClatter
    },
    // 2: Space Rocket (F3) - Thruster roar (ICE disabled)
    {
        EngineConfiguration::SciFiWarp,
        ForcedInductionType::None,
        30.0, 120.0, 0.50, 0.50, 0.00, 100.0, 300.0, 0.20, 3000.0, 2.0, false, 0.0, false, false, false
    },
    // 3: Helicopter (F4) - Turboshaft turbine hum & rotor chop
    {
        EngineConfiguration::Inline4,
        ForcedInductionType::None,
        180.0, 420.0, 0.40, 0.60, 0.00, 220.0, 680.0, 0.50, 4800.0, 2.0, false, 0.0, false, false, false
    },
    // 4: Hovercraft (F5) - Ducted air fans and skirt hiss
    {
        EngineConfiguration::V6,
        ForcedInductionType::None,
        75.0, 220.0, 0.55, 0.45, 0.00, 160.0, 480.0, 0.40, 4200.0, 2.0, false, 0.0, false, false, false
    },
    // 5: Tracked Tank (F6) - Heavy 12-Cylinder Turbo Diesel, track clatter, air brakes
    {
        EngineConfiguration::HeavyDieselV12,
        ForcedInductionType::Turbocharger,
        38.0,   // idleFrequency
        165.0,  // redlineFrequency
        0.75,   // cylinderHarmonicWeight
        0.25,   // subHarmonicWeight
        0.45,   // dieselKnockIntensity
        120.0,  // exhaustResonanceFreq1
        440.0,  // exhaustResonanceFreq2
        0.80,   // turboSpoolRate
        3800.0, // turboWhistleMaxFreq
        2.00,   // superchargerGearRatio
        false,  // hasOverrunPops
        0.00,   // overrunPopIntensity
        false,  // hasStraightCutGearbox
        true,   // hasPneumaticAirBrakes
        true    // hasTrackCrawlClatter
    },
    // 6: Anti-Gravity (F7) - Sci-Fi FM Dual-Oscillator Warp
    {
        EngineConfiguration::SciFiWarp,
        ForcedInductionType::None,
        120.0, 850.0, 0.50, 0.50, 0.00, 250.0, 850.0, 0.25, 6500.0, 2.0, false, 0.0, false, false, false
    },
    // 7: Drift Car (F8) - Twin-Turbo Crossplane V8, loud spool, BOV flutter, violent backfires
    {
        EngineConfiguration::CrossplaneV8,
        ForcedInductionType::Turbocharger,
        60.0,   // idleFrequency
        390.0,  // redlineFrequency
        0.80,   // cylinderHarmonicWeight
        0.20,   // subHarmonicWeight
        0.00,   // dieselKnockIntensity
        140.0,  // exhaustResonanceFreq1
        620.0,  // exhaustResonanceFreq2
        0.35,   // turboSpoolRate
        5600.0, // turboWhistleMaxFreq
        2.40,   // superchargerGearRatio
        true,   // hasOverrunPops
        0.90,   // overrunPopIntensity
        true,   // hasStraightCutGearbox
        false,  // hasPneumaticAirBrakes
        false   // hasTrackCrawlClatter
    },
    // 8: Monster Truck (F9) - Supercharged 540ci Big Block V8, Roots blower whine, deep burble
    {
        EngineConfiguration::CrossplaneV8,
        ForcedInductionType::Supercharger,
        45.0,   // idleFrequency
        280.0,  // redlineFrequency
        0.75,   // cylinderHarmonicWeight
        0.25,   // subHarmonicWeight
        0.00,   // dieselKnockIntensity
        110.0,  // exhaustResonanceFreq1
        480.0,  // exhaustResonanceFreq2
        0.40,   // turboSpoolRate
        4800.0, // turboWhistleMaxFreq
        2.45,   // superchargerGearRatio
        true,   // hasOverrunPops
        0.50,   // overrunPopIntensity
        false,  // hasStraightCutGearbox
        false,  // hasPneumaticAirBrakes
        false   // hasTrackCrawlClatter
    },
    // 9: Semi-Truck (F10) - 15L Heavy Turbo Diesel, slow spool, air brake purge
    {
        EngineConfiguration::HeavyDieselI6,
        ForcedInductionType::Turbocharger,
        32.0,   // idleFrequency
        135.0,  // redlineFrequency
        0.80,   // cylinderHarmonicWeight
        0.20,   // subHarmonicWeight
        0.55,   // dieselKnockIntensity
        105.0,  // exhaustResonanceFreq1
        410.0,  // exhaustResonanceFreq2
        0.90,   // turboSpoolRate
        3600.0, // turboWhistleMaxFreq
        2.00,   // superchargerGearRatio
        false,  // hasOverrunPops
        0.00,   // overrunPopIntensity
        false,  // hasStraightCutGearbox
        true,   // hasPneumaticAirBrakes
        false   // hasTrackCrawlClatter
    },
    // 10: Forklift (F11) - Electric DC Motor PWM whine, tight scrub
    {
        EngineConfiguration::ElectricPWM,
        ForcedInductionType::None,
        80.0,   // idleFrequency
        420.0,  // redlineFrequency
        0.30,   // cylinderHarmonicWeight
        0.70,   // subHarmonicWeight
        0.00,   // dieselKnockIntensity
        300.0,  // exhaustResonanceFreq1
        1200.0, // exhaustResonanceFreq2
        0.20,   // turboSpoolRate
        2400.0, // turboWhistleMaxFreq
        1.00,   // superchargerGearRatio
        false,  // hasOverrunPops
        0.00,   // overrunPopIntensity
        false,  // hasStraightCutGearbox
        false,  // hasPneumaticAirBrakes
        false   // hasTrackCrawlClatter
    },
    // 11: Quadcopter Drone (F12) - High-KV Brushless Buzz
    {
        EngineConfiguration::Inline4,
        ForcedInductionType::None,
        270.0, 650.0, 0.40, 0.60, 0.00, 350.0, 950.0, 0.20, 5000.0, 1.0, false, 0.0, false, false, false
    }
}};

} // namespace

const VehicleAcousticProfile& AcousticProfileRegistry::getProfile(int vehicleIndex) noexcept
{
    const int clampedIndex { std::clamp(vehicleIndex, 0, static_cast<int>(kFleetProfiles.size() - 1U)) };
    return kFleetProfiles[static_cast<std::size_t>(clampedIndex)];
}

} // namespace vehicle_sim::audio
