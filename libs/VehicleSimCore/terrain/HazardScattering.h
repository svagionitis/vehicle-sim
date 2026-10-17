#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <vector>

#include "libs/VehicleSimCore/math/Vec2.h"
#include "libs/VehicleSimCore/terrain/PoissonDiskSampler.h"
#include "libs/VehicleSimCore/terrain/SurfaceMaterial.h"

namespace vehicle_sim::physics {
class PhysicsWorld2D;
class RigidBody2D;
class Collider2D;
} // namespace vehicle_sim::physics

namespace vehicle_sim::terrain {

struct CircuitTrack2D;
class SurfaceMap2D;

/// @enum HazardType
/// @brief Identifies the physical and behavioral archetype of a scattered track obstacle.
enum class HazardType : std::uint8_t {
    TrafficCone,    ///< Lightweight movable dynamic obstacle.
    OilSlick,       ///< Low-friction surface material patch.
    StandingPuddle, ///< Standing water patch causing hydrodynamic drag and hydroplaning.
    Rock,           ///< Heavy static rigid body collision obstacle.
    BoostPad,       ///< Forward kinetic impulse velocity trigger.
    TireBarrier     ///< High-damping perimeter impact attenuator.
};

/// @enum HazardZonePolicy
/// @brief Specifies topological placement constraints relative to track ribbon and shoulders.
enum class HazardZonePolicy : std::uint8_t {
    TrackRibbonOnly,  ///< Scattered strictly on drivable asphalt/tarmac ribbon.
    ShoulderOnly,     ///< Scattered strictly in loose gravel/grass runoff shoulders.
    TrackAndShoulder, ///< Scattered across both ribbon and runoff shoulders.
    TrackBorders      ///< Scattered along the dividing boundary edge between ribbon and shoulder.
};

/// @struct ScatteredHazard
/// @brief Fully described track obstacle with spatial, physical, and topological parameters.
struct ScatteredHazard {
    math::Vec2 position { 0.0, 0.0 }; ///< 2D world coordinates in meters.
    double orientation { 0.0 }; ///< Yaw angle in radians.
    double boundingRadius { 0.5 }; ///< Geometric collision / clearance radius in meters.
    double trackProgress { 0.0 }; ///< Normalized progress along centerline in [0, 1].
    double lateralOffset { 0.0 }; ///< Signed orthogonal distance from centerline in meters.
    HazardType type { HazardType::TrafficCone }; ///< Hazard physical archetype.
    double severity { 1.0 }; ///< Intensity modifier (e.g. friction factor or boost magnitude).
};

/// @struct HazardScatteringConfig
/// @brief Configuration settings controlling procedural track hazard generation.
struct HazardScatteringConfig {
    double minDistance { 6.0 }; ///< Minimum separation distance between any two hazards in meters.
    HazardZonePolicy zonePolicy { HazardZonePolicy::TrackAndShoulder }; ///< Zone placement rule.
    double coneProbability { 0.35 }; ///< Relative weight of traffic cones.
    double oilSlickProbability { 0.20 }; ///< Relative weight of oil slicks.
    double puddleProbability { 0.20 }; ///< Relative weight of standing water puddles.
    double rockProbability { 0.15 }; ///< Relative weight of heavy rocks.
    double boostPadProbability { 0.10 }; ///< Relative weight of speed boost pads.
    double startClearanceDistance { 30.0 }; ///< Length in meters after starting line kept hazard-free.
    double finishClearanceDistance { 20.0 }; ///< Length in meters before finish line kept hazard-free.
    std::size_t maxHazardCount { 100U }; ///< Maximum allowable scattered hazards clamp.
    std::uint32_t seed { 4242U }; ///< Deterministic PRNG seed.
};

/// @struct HazardPhysicalProps
/// @brief Holds RAII-managed physical rigid bodies and colliders instantiated for hazards.
struct HazardPhysicalProps {
    HazardPhysicalProps();
    ~HazardPhysicalProps();
    HazardPhysicalProps(HazardPhysicalProps&&) noexcept;
    HazardPhysicalProps& operator=(HazardPhysicalProps&&) noexcept;
    HazardPhysicalProps(const HazardPhysicalProps&) = delete;
    HazardPhysicalProps& operator=(const HazardPhysicalProps&) = delete;

    std::vector<std::unique_ptr<physics::RigidBody2D>> bodies { }; ///< Managed rigid bodies.
    std::vector<std::unique_ptr<physics::Collider2D>> colliders { }; ///< Managed colliders.
};

/// @class HazardScattering
/// @brief Coordinates procedural blue-noise hazard placement along track corridors.
class HazardScattering {
public:
    HazardScattering() = delete;

    /// @brief Generates scattered hazards along a 2D closed circuit track.
    /// @param[in] track Circuit track geometry and centerline spline.
    /// @param[in] config Hazard scattering rules and probabilities.
    /// @return Array of placed and classified ScatteredHazard instances.
    [[nodiscard]] static std::vector<ScatteredHazard> scatterOnTrack(
        const CircuitTrack2D& track, const HazardScatteringConfig& config);

    /// @brief Stamps surface-modifying hazards (oil, water, boost) into a 2D surface map.
    /// @param[in,out] surfaceMap Target terrain surface map to mutate.
    /// @param[in] hazards Array of scattered hazards.
    static void stampSurfaceMaterials(
        SurfaceMap2D& surfaceMap, const std::vector<ScatteredHazard>& hazards);

    /// @brief Instantiates physical obstacle colliders (rocks, cones) into a physics world.
    /// @param[in,out] world Target physics world receiving rigid bodies.
    /// @param[in] hazards Array of scattered hazards.
    /// @param[out] outProps Container receiving RAII-managed rigid bodies and colliders.
    static void spawnPhysicsColliders(
        physics::PhysicsWorld2D& world,
        const std::vector<ScatteredHazard>& hazards,
        HazardPhysicalProps& outProps);
};

} // namespace vehicle_sim::terrain
