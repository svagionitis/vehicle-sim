#pragma once

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

#include "libs/VehicleSimCore/dynamics/SideViewCarDynamics.h"
#include "libs/VehicleSimCore/math/Vec2.h"
#include "libs/VehicleSimCore/terrain/BankedTrack.h"
#include "libs/VehicleSimCore/terrain/HazardScattering.h"
#include "libs/VehicleSimCore/terrain/Heightfield1D.h"
#include "libs/VehicleSimCore/terrain/SurfaceMap2D.h"
#include "libs/VehicleSimCore/terrain/SurfaceMaterial.h"

namespace vehicle_sim::terrain {

/// @struct TrialsCourseConfig
/// @brief Configuration parameters for procedural trials obstacle courses.
/// @details Specifies geometric dimensions for start ramp, undulating camel humps,
/// ascending step ledges, steep hill climb, rocky terrain roughness, and finish platform.
struct TrialsCourseConfig {
    double startLength { 15.0 }; ///< Horizontal flat starting pad length in meters.
    double humpSpacing { 6.0 }; ///< Wavelength of undulating camel humps in meters.
    double humpHeight { 1.2 }; ///< Peak elevation amplitude of humps in meters.
    std::size_t humpCount { 4 }; ///< Number of camel humps in sequence.
    double stepLedgeHeight { 1.5 }; ///< Vertical elevation step for step-up ledges in meters.
    double stepLedgeWidth { 5.0 }; ///< Horizontal flat tread width of step ledges in meters.
    std::size_t stepLedgeCount { 2 }; ///< Number of ascending step-up ledges.
    double climbSlope { 0.45 }; ///< Steep mountain climb slope gradient (rise/run).
    double climbLength { 20.0 }; ///< Horizontal length of steep climb in meters.
    double roughLength { 25.0 }; ///< Horizontal length of rough rocky section in meters.
    double roughAmplitude { 0.5 }; ///< High-frequency rocky surface noise amplitude in meters.
    double finishLength { 15.0 }; ///< Flat finish landing platform length in meters.
    std::uint32_t seed { 1337U }; ///< Deterministic pseudorandom seed.
};

/// @struct TrialsCourse
/// @brief Procedurally generated obstacle course for trials bikes and side-view physics.
struct TrialsCourse {
    Heightfield1D terrain { }; ///< Procedural 1D continuous heightfield profile.
    math::Vec2 startPosition { 0.0, 0.0 }; ///< Initial spawn coordinate [x, y] in meters.
    math::Vec2 finishPosition { 0.0, 0.0 }; ///< Finish gate coordinate [x, y] in meters.
    double totalLength { 0.0 }; ///< Total horizontal course extent in meters.
    double finishHeight { 0.0 }; ///< Elevation at finish gate in meters.

    /// @brief Populates SideViewCarDynamics platform collection.
    /// @param[in,out] car Target side-view car dynamics instance.
    /// @param[in] config Platform extraction parameters.
    /// @param[in] clearExisting True to clear prior platforms before appending.
    void populateSideViewCar(
        dynamics::SideViewCarDynamics& car, const PlatformExtractionConfig& config, bool clearExisting = true) const;
};

/// @struct PlatformerLevelConfig
/// @brief Configuration parameters for multi-tiered 2D platformer stages.
/// @details Defines tier spacing, platform lengths, hazard gap intervals, and surface hazard probabilities.
struct PlatformerLevelConfig {
    double levelWidth { 120.0 }; ///< Horizontal extent in meters [-width/2, width/2].
    double tierHeight { 3.5 }; ///< Vertical elevation spacing between tiers in meters.
    std::size_t tierCount { 4 }; ///< Number of vertical tiers above ground.
    double platformMinLength { 8.0 }; ///< Minimum horizontal length of floating platform in meters.
    double platformMaxLength { 16.0 }; ///< Maximum horizontal length of floating platform in meters.
    double gapMinLength { 3.0 }; ///< Minimum hazard air gap between platforms in meters.
    double gapMaxLength { 6.0 }; ///< Maximum hazard air gap between platforms in meters.
    double groundY { 0.0 }; ///< Ground floor elevation in meters.
    bool hasGroundFloor { true }; ///< True if ground tier 0 is continuous base floor.
    bool oneWayPlatforms { true }; ///< True if floating tiers allow upward jump pass-through.
    double hazardProbability { 0.25 }; ///< Probability in [0, 1] of platform having hazard material.
    std::uint32_t seed { 42U }; ///< Deterministic pseudorandom seed.
};

/// @struct PlatformerLevel
/// @brief Multi-tiered floating platform layout for SideViewCarDynamics.
struct PlatformerLevel {
    std::vector<dynamics::SideViewPlatform> platforms { }; ///< All generated platform segments.
    math::Vec2 spawnPoint { 0.0, 0.0 }; ///< Player starting spawn position in meters.
    math::Vec2 goalPoint { 0.0, 0.0 }; ///< Topmost goal platform location in meters.
    std::size_t tierCount { 0 }; ///< Number of generated platform tiers.

    /// @brief Injects all platforms directly into SideViewCarDynamics.
    /// @param[in,out] car Target side-view car dynamics instance.
    /// @param[in] clearExisting True to clear prior platforms before appending.
    void populateSideViewCar(dynamics::SideViewCarDynamics& car, bool clearExisting = true) const;
};

/// @struct HillClimbConfig
/// @brief Configuration parameters for extreme procedural hill climb courses.
/// @details Specifies horizontal run, base slope gradient, fractal mountain roughness,
/// high-incline steep pitches, mud bogs, and summit ice patches.
struct HillClimbConfig {
    double totalLength { 300.0 }; ///< Horizontal length of mountain run in meters.
    double baseSlope { 0.20 }; ///< Nominal upward slope gradient dy/dx (~11.3 deg).
    double noiseAmplitude { 4.0 }; ///< Mountain fractal noise elevation amplitude in meters.
    double noiseFrequency { 0.02 }; ///< Mountain noise spatial frequency in 1/m.
    std::size_t steepPitchCount { 3 }; ///< Number of high-incline steep pitch segments.
    double steepSlope { 0.65 }; ///< Steep pitch slope gradient dy/dx (~33 deg).
    double steepLength { 15.0 }; ///< Horizontal length of each steep pitch in meters.
    std::size_t mudZoneCount { 2 }; ///< Number of low-traction mud bog intervals.
    std::size_t iceZoneCount { 2 }; ///< Number of high-altitude ice patches.
    std::uint32_t seed { 9876U }; ///< Deterministic pseudorandom seed.
};

/// @struct HillClimbCourse
/// @brief Extreme procedural mountain hill climb course with continuous heightfield.
struct HillClimbCourse {
    Heightfield1D terrain { }; ///< Procedural continuous mountain heightfield.
    math::Vec2 startPosition { 0.0, 0.0 }; ///< Base mountain spawn position in meters.
    math::Vec2 peakPosition { 0.0, 0.0 }; ///< Summit peak position in meters.
    double totalElevationGain { 0.0 }; ///< Total vertical elevation gain in meters.
    double maxSlopeAngle { 0.0 }; ///< Steepest surface slope angle in radians.

    /// @brief Populates SideViewCarDynamics platform collection.
    /// @param[in,out] car Target side-view car dynamics instance.
    /// @param[in] config Platform extraction parameters.
    /// @param[in] clearExisting True to clear prior platforms before appending.
    void populateSideViewCar(
        dynamics::SideViewCarDynamics& car, const PlatformExtractionConfig& config, bool clearExisting = true) const;
};

/// @struct CircuitTrackConfig
/// @brief Configuration parameters for 2D top-down racing circuit tracks.
/// @details Defines perimeter radius, waypoint count, roadway ribbon width, runoff shoulders,
/// procedural shape perturbation, elevation variations, banking superelevation, and surface materials.
struct CircuitTrackConfig {
    double radiusX { 100.0 }; ///< Semi-major radius of track loop in meters.
    double radiusY { 70.0 }; ///< Semi-minor radius of track loop in meters.
    std::size_t waypointCount { 24 }; ///< Number of perimeter spline control nodes.
    double trackWidth { 10.0 }; ///< Drivable roadway ribbon width in meters.
    double shoulderWidth { 4.0 }; ///< Loose runoff shoulder width in meters.
    double perturbation { 0.25 }; ///< Radial shape distortion fraction in [0, 0.4].
    double elevationVariation { 6.0 }; ///< Peak vertical undulating elevation in meters.
    bool isClosed { true }; ///< True if track forms closed continuous loop.
    bool enableBanking { true }; ///< True to generate physical transversal banking and superelevation.
    TrackCamberType camberType { TrackCamberType::Superelevation }; ///< Cross-section camber profile.
    double designSpeed { 25.0 }; ///< Target cornering design speed in m/s (~90 km/h).
    double maxBankAngle { 0.40 }; ///< Maximum bank angle clamp in radians (~22.9 deg).
    double crownSlope { 0.02 }; ///< Drainage cross-fall on straights (e.g. 2% slope).
    double transitionLength { 25.0 }; ///< Superelevation runoff easement length in meters.
    double bowlCurvature { 0.05 }; ///< Quadratic coefficient for ParabolicBowl profile.
    SurfaceMaterial roadMaterial { SurfaceMaterial::getPreset(MaterialType::Tarmac) };
    SurfaceMaterial shoulderMaterial { SurfaceMaterial::getPreset(MaterialType::Gravel) };
    SurfaceMaterial offtrackMaterial { SurfaceMaterial::getPreset(MaterialType::Grass) };
    std::uint32_t seed { 2024U }; ///< Deterministic pseudorandom seed.
    bool enableHazards { false }; ///< True to scatter procedural obstacles along track corridor.
    HazardScatteringConfig hazardConfig { }; ///< Configuration for scattered hazards.
};

/// @struct CircuitTrack2D
/// @brief Procedural 2D ribbon circuit track with SurfaceMap2D integration, banking, and centerline queries.
struct CircuitTrack2D {
    SurfaceMap2D surfaceMap { }; ///< 2D top-down elevation and surface material map.
    std::vector<math::Vec2> centerline { }; ///< Discrete roadway centerline polyline vertices.
    std::vector<math::Vec2> leftBoundary { }; ///< Left boundary edge polyline vertices.
    std::vector<math::Vec2> rightBoundary { }; ///< Right boundary edge polyline vertices.
    std::vector<double> bankAngles { }; ///< Local bank angle theta at each centerline vertex in radians.
    std::vector<math::Vec2> checkpoints { }; ///< Timing checkpoint coordinates in order.
    math::Vec2 startGridPosition { 0.0, 0.0 }; ///< Starting grid coordinate in meters.
    double startHeading { 0.0 }; ///< Vehicle starting orientation angle in radians.
    double trackLength { 0.0 }; ///< Total centerline track perimeter length in meters.
    double trackWidth { 10.0 }; ///< Drivable roadway ribbon width in meters.
    double shoulderWidth { 4.0 }; ///< Runoff shoulder width in meters.
    bool isClosed { true }; ///< True if circuit forms closed loop.
    BankedTrackProfile camberProfile { }; ///< Active camber and banking evaluator.
    std::vector<ScatteredHazard> hazards { }; ///< Procedural scattered hazards along track.

    /// @brief Gets total number of active scattered hazards.
    /// @return Count of hazards.
    [[nodiscard]] std::size_t getHazardCount() const noexcept;

    /// @brief Clears all scattered hazards from track.
    void clearHazards() noexcept;

    /// @brief Tests if world coordinate lies within drivable roadway ribbon.
    /// @param[in] point World query position in meters.
    /// @return True if distance from centerline is <= half track width.
    [[nodiscard]] bool isPointOnTrack(const math::Vec2& point) const noexcept;

    /// @brief Computes shortest Euclidean distance from point to centerline.
    /// @param[in] point World query position in meters.
    /// @return Orthogonal distance in meters to closest centerline segment.
    [[nodiscard]] double getDistanceToCenter(const math::Vec2& point) const noexcept;

    /// @brief Computes fractional progress along track centerline in range [0, 1].
    /// @param[in] point World query position in meters.
    /// @return Fractional track progress [0.0, 1.0].
    [[nodiscard]] double getTrackProgress(const math::Vec2& point) const noexcept;

    /// @brief Evaluates interpolated bank angle at world query coordinates.
    /// @param[in] point World query position in meters.
    /// @return Bank angle in radians (0.0 if off track).
    [[nodiscard]] double getBankAngle(const math::Vec2& point) const noexcept;

    /// @brief Evaluates bank angle at normalized track progress in [0, 1].
    /// @param[in] progress Fractional progress along track [0.0, 1.0].
    /// @return Bank angle in radians.
    [[nodiscard]] double getBankAngleAtProgress(double progress) const noexcept;

    /// @brief Evaluates full cross-section sample at world query position.
    /// @param[in] point World query position in meters.
    /// @return Evaluated CamberProfileSample.
    [[nodiscard]] CamberProfileSample sampleCamber(const math::Vec2& point) const noexcept;
};

/// @class TrackGenerator
/// @brief Procedural course and track generator for side-view and top-down vehicle simulation.
/// @details Synthesizes trials courses with obstacle features, multi-tiered platformer stages,
/// extreme mountain hill climbs, and 2D closed-loop racing circuits.
///
/// ```
///  ASCII Diagram: Procedural Track Generator Pipelines
///
///  +-------------------------------------------------------------------------+
///  |                             TrackGenerator                              |
///  +-------------------+--------------------+-------------------+------------+
///  | generateTrials    | generatePlatformer | generateHillClimb | generate   |
///  | Course            | Level              |                   | Circuit    |
///  +---------+---------+---------+----------+---------+---------+-----+------+
///            |                   |                    |               |
///            v                   v                    v               v
///      TrialsCourse       PlatformerLevel      HillClimbCourse  CircuitTrack2D
///      (Heightfield1D)    (SideViewPlatform)   (Heightfield1D)  (SurfaceMap2D)
///            |                   |                    |               |
///            v                   v                    v               v
///     TrialsBikeDynamics  SideViewCarDynamics  TrialsBike/Car   Arcade/MonsterCar
/// ```
///
/// ```mermaid
/// graph TD
///     subgraph GeneratorFactory ["TrackGenerator Factory"]
///         GT["generateTrialsCourse()"]
///         GP["generatePlatformerLevel()"]
///         GH["generateHillClimb()"]
///         GC["generateCircuitTrack()"]
///     end
///     subgraph Results ["Generated Track Output"]
///         TC["TrialsCourse"]
///         PL["PlatformerLevel"]
///         HC["HillClimbCourse"]
///         CT["CircuitTrack2D"]
///     end
///     subgraph PhysicsIntegration ["Physics Dynamics"]
///         TB["TrialsBikeDynamics"]
///         SV["SideViewCarDynamics"]
///         AC["ArcadeCarDynamics"]
///     end
///     GT --> TC
///     GP --> PL
///     GH --> HC
///     GC --> CT
///     TC --> TB
///     PL --> SV
///     HC --> TB
///     CT --> AC
/// ```
class TrackGenerator {
public:
    TrackGenerator() = delete;

    /// @brief Generates procedural trials obstacle motorcycle course.
    /// @param[in] config Course geometric parameters.
    /// @return Configured TrialsCourse instance.
    [[nodiscard]] static TrialsCourse generateTrialsCourse(const TrialsCourseConfig& config);

    /// @brief Generates multi-tiered floating platformer stage for SideViewCarDynamics.
    /// @param[in] config Level generation parameters.
    /// @return Configured PlatformerLevel instance.
    [[nodiscard]] static PlatformerLevel generatePlatformerLevel(const PlatformerLevelConfig& config);

    /// @brief Generates extreme procedural mountain hill climb course.
    /// @param[in] config Hill climb generation parameters.
    /// @return Configured HillClimbCourse instance.
    [[nodiscard]] static HillClimbCourse generateHillClimb(const HillClimbConfig& config);

    /// @brief Generates procedural 2D closed-loop or sprint circuit track.
    /// @param[in] config Circuit geometric configuration.
    /// @return Configured CircuitTrack2D instance with surface map and spline boundaries.
    [[nodiscard]] static CircuitTrack2D generateCircuitTrack(const CircuitTrackConfig& config);
};

} // namespace vehicle_sim::terrain
