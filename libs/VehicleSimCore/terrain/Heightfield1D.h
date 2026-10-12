#pragma once

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <string_view>
#include <vector>

#include "libs/VehicleSimCore/dynamics/SideViewCarDynamics.h"
#include "libs/VehicleSimCore/math/Vec2.h"
#include "libs/VehicleSimCore/terrain/FractalNoise.h"
#include "libs/VehicleSimCore/terrain/SurfaceMaterial.h"

namespace vehicle_sim::terrain {

/// @enum HeightfieldInterp
/// @brief Spatial interpolation algorithm between discrete elevation samples.
enum class HeightfieldInterp : std::uint8_t {
    Linear, ///< Piecewise linear C0 continuous interpolation.
    CubicHermite ///< Smooth Catmull-Rom / Hermite C1 continuous spline interpolation.
};

/// @enum HeightfieldBoundary
/// @brief Out-of-bounds coordinate handling policy for discrete samples.
enum class HeightfieldBoundary : std::uint8_t {
    Clamp, ///< Clamp evaluation coordinate to closest defined boundary sample.
    Extrapolate, ///< Linearly extrapolate elevation using boundary segment slope.
    Zero ///< Return zero elevation outside the defined domain.
};

/// @struct HeightSample1D
/// @brief Discrete elevation control point along the 1D horizontal axis.
struct HeightSample1D {
    double x { 0.0 }; ///< Horizontal spatial coordinate in meters.
    double y { 0.0 }; ///< Vertical surface elevation in meters.
};

/// @struct SurfaceSample1D
/// @brief Complete geometric and physical surface sample at coordinate x.
struct SurfaceSample1D {
    double elevation { 0.0 }; ///< Surface elevation y = h(x) in meters.
    double derivative { 0.0 }; ///< First-order spatial derivative dy/dx (tangent slope).
    double slopeAngle { 0.0 }; ///< Surface inclination angle in radians [-pi/2, pi/2].
    math::Vec2 normal { 0.0, 1.0 }; ///< Upward unit surface normal vector.
    math::Vec2 tangent { 1.0, 0.0 }; ///< Forward unit surface tangent vector.
    SurfaceMaterial material {}; ///< Surface friction and drag physical parameters.
};

/// @struct MaterialZone
/// @brief Spatial interval override for surface physical material properties.
struct MaterialZone {
    double minX { 0.0 }; ///< Minimum horizontal interval bound in meters.
    double maxX { 0.0 }; ///< Maximum horizontal interval bound in meters.
    SurfaceMaterial material {}; ///< Surface material applied within interval.
};

/// @enum PlatformExtractionMethod
/// @brief Strategy used to extract discrete platform rungs from continuous terrain.
enum class PlatformExtractionMethod : std::uint8_t {
    FlatPlateau, ///< Extracts contiguous flat plateaus where |dy/dx| <= maxSlope.
    StepQuantization ///< Quantizes elevation into discrete stepped horizontal tiers.
};

/// @struct PlatformExtractionConfig
/// @brief Parameter configuration for extracting discrete platform rungs.
struct PlatformExtractionConfig {
    double minX { -100.0 }; ///< Extraction range minimum X coordinate in meters.
    double maxX { 100.0 }; ///< Extraction range maximum X coordinate in meters.
    double sampleStep { 0.25 }; ///< Spatial sampling step dx for evaluation in meters.
    double maxSlope { 0.08 }; ///< Maximum absolute slope (|dy/dx|) considered flat (~4.6 deg).
    double minWidth { 1.5 }; ///< Minimum platform length (maxX - minX) in meters.
    double mergeElevationTolerance { 0.15 }; ///< Max delta Y in meters to merge adjacent flat spans.
    double tierHeight { 2.0 }; ///< Elevation step for StepQuantization mode in meters.
    bool defaultOneWay { true }; ///< Flag assigned to extracted platform oneWay property.
    PlatformExtractionMethod method { PlatformExtractionMethod::FlatPlateau }; ///< Extraction algorithm.
};

/// @struct HeightfieldPlatform
/// @brief Extracted horizontal platform segment with associated surface material.
struct HeightfieldPlatform {
    double y { 0.0 }; ///< Surface elevation in meters.
    double minX { 0.0 }; ///< Minimum horizontal boundary in meters.
    double maxX { 0.0 }; ///< Maximum horizontal boundary in meters.
    double friction { 1.0 }; ///< Surface Coulomb contact friction coefficient.
    bool oneWay { true }; ///< True if platform allows upward pass-through.
    MaterialType materialType { MaterialType::Tarmac }; ///< Associated material preset.

    /// @brief Converts platform specification into dynamics-layer SideViewPlatform.
    /// @return Equivalent SideViewPlatform struct.
    [[nodiscard]] dynamics::SideViewPlatform toSideViewPlatform() const noexcept;
};

/// @class Heightfield1D
/// @brief 1D heightfield terrain supporting discrete interpolation, procedural noise, and platform extraction.
/// @details Combines hand-authored discrete elevation control points, procedural multi-octave fractal noise,
/// analytical first derivatives, upward surface normals, material zones, and automated platform extraction
/// for side-view vehicle physics simulation.
///
/// ```
///  ASCII Diagram: 1D Surface Profile & Platform Extraction
///
///   Elevation Y
///        ^                Normal n
///        |                   ^
///        |                   |
///        |             .---*-*---.  Tangent t --->
///        |            /           \                      Plateau Platform [minX, maxX]
///        |           /  Hilltop    \                    ===============================
///        |   .------'               '--------.         /
///        |  / Plateau                         \       /
///        | /                                   '-----'
///        +----------------------------------------------------> Horizontal X
///          [ Zone A: Dirt ]   [ Zone B: Ice ]
/// ```
///
/// ```mermaid
/// graph TD
///     subgraph EvaluationPipeline ["Evaluation Pipeline"]
///         XCoord["Coordinate x"] --> InterpNode["Sample Interpolation (Linear / Cubic Hermite)"]
///         XCoord --> NoiseNode["Fractal Noise Synthesis (fBm / Billow / Ridged)"]
///         InterpNode --> SumNode["Combined Elevation & Derivative dy/dx"]
///         NoiseNode --> SumNode
///         SumNode --> NormalTangent["Normal n & Tangent t Calculation"]
///         XCoord --> MaterialZones["Material Zone Lookup"]
///         NormalTangent --> SurfaceSample["SurfaceSample1D"]
///         MaterialZones --> SurfaceSample
///     end
///     subgraph ExtractionPipeline ["Platform Extraction"]
///         SurfaceSample --> FilterNode["Slope / Tier Threshold Filtering"]
///         FilterNode --> MergeNode["Collinear Span Merging & Width Validation"]
///         MergeNode --> ExtractedPlatforms["HeightfieldPlatform / SideViewPlatform"]
///     end
/// ```
class Heightfield1D {
public:
    /// @brief Default constructor with baseline flat tarmac profile.
    Heightfield1D() noexcept = default;

    /// @brief Sets baseline uniform vertical elevation.
    /// @param[in] baseHeight Base elevation in meters.
    void setBaseHeight(double baseHeight) noexcept;

    /// @brief Gets baseline uniform vertical elevation.
    /// @return Base elevation in meters.
    [[nodiscard]] double getBaseHeight() const noexcept;

    /// @brief Sets procedural noise scaling and offset parameters.
    /// @param[in] verticalScale Amplitude multiplier for procedural elevation in meters.
    /// @param[in] horizontalScale Spatial frequency multiplier along X axis.
    /// @param[in] horizontalOffset Spatial offset translation along X axis in meters.
    void setNoiseTransform(double verticalScale, double horizontalScale, double horizontalOffset) noexcept;

    /// @brief Enables or disables procedural fractal noise evaluation.
    /// @param[in] enable True to enable procedural noise contribution.
    void enableNoise(bool enable) noexcept;

    /// @brief Queries whether procedural fractal noise is enabled.
    /// @return True if procedural noise is active.
    [[nodiscard]] bool isNoiseEnabled() const noexcept;

    /// @brief Sets the procedural fractal noise synthesizer configuration.
    /// @param[in] config Fractal noise configuration parameters.
    void setNoiseConfig(const FractalConfig& config) noexcept;

    /// @brief Gets active procedural fractal noise synthesizer.
    /// @return Const reference to internal FractalNoise object.
    [[nodiscard]] const FractalNoise& getNoise() const noexcept;

    /// @brief Sets discrete elevation control samples.
    /// @details Automatically sorts control points in ascending X order.
    /// @param[in] samples Collection of discrete elevation points.
    void setSamples(const std::vector<HeightSample1D>& samples);

    /// @brief Appends a single discrete elevation control sample.
    /// @param[in] sample New control point.
    void addSample(const HeightSample1D& sample);

    /// @brief Clears all discrete elevation control samples.
    void clearSamples() noexcept;

    /// @brief Gets discrete elevation control sample count.
    /// @return Number of stored samples.
    [[nodiscard]] std::size_t getSampleCount() const noexcept;

    /// @brief Gets stored discrete elevation control samples.
    /// @return Const reference to sample vector.
    [[nodiscard]] const std::vector<HeightSample1D>& getSamples() const noexcept;

    /// @brief Sets discrete sample spatial interpolation algorithm.
    /// @param[in] interp Target interpolation method (Linear or CubicHermite).
    void setInterpolation(HeightfieldInterp interp) noexcept;

    /// @brief Gets discrete sample spatial interpolation algorithm.
    /// @return Current interpolation mode.
    [[nodiscard]] HeightfieldInterp getInterpolation() const noexcept;

    /// @brief Sets discrete sample out-of-bounds boundary handling mode.
    /// @param[in] boundary Target boundary policy (Clamp, Extrapolate, Zero).
    void setBoundaryMode(HeightfieldBoundary boundary) noexcept;

    /// @brief Gets discrete sample out-of-bounds boundary handling mode.
    /// @return Current boundary policy.
    [[nodiscard]] HeightfieldBoundary getBoundaryMode() const noexcept;

    /// @brief Sets default surface physical material.
    /// @param[in] material Material properties used outside defined zones.
    void setDefaultMaterial(const SurfaceMaterial& material) noexcept;

    /// @brief Gets default surface physical material.
    /// @return Current default surface material.
    [[nodiscard]] const SurfaceMaterial& getDefaultMaterial() const noexcept;

    /// @brief Adds a spatial interval override for surface material.
    /// @param[in] minX Minimum horizontal coordinate in meters.
    /// @param[in] maxX Maximum horizontal coordinate in meters.
    /// @param[in] material Surface physical material properties.
    void addMaterialZone(double minX, double maxX, const SurfaceMaterial& material);

    /// @brief Adds a spatial interval override using a material preset.
    /// @param[in] minX Minimum horizontal coordinate in meters.
    /// @param[in] maxX Maximum horizontal coordinate in meters.
    /// @param[in] materialType Target material preset type.
    void addMaterialZone(double minX, double maxX, MaterialType materialType);

    /// @brief Removes all registered material interval overrides.
    void clearMaterialZones() noexcept;

    /// @brief Gets registered material zone count.
    /// @return Number of registered material zones.
    [[nodiscard]] std::size_t getMaterialZoneCount() const noexcept;

    /// @brief Evaluates surface elevation y = h(x) at coordinate x.
    /// @param[in] x Horizontal spatial coordinate in meters.
    /// @return Surface elevation in meters.
    [[nodiscard]] double getHeight(double x) const noexcept;

    /// @brief Evaluates first-order surface slope derivative dy/dx at coordinate x.
    /// @param[in] x Horizontal spatial coordinate in meters.
    /// @return Slope gradient dy/dx.
    [[nodiscard]] double getSlope(double x) const noexcept;

    /// @brief Evaluates terrain slope inclination angle at coordinate x.
    /// @param[in] x Horizontal spatial coordinate in meters.
    /// @return Inclination angle in radians in range [-pi/2, pi/2].
    [[nodiscard]] double getSlopeAngle(double x) const noexcept;

    /// @brief Evaluates unit upward surface normal vector at coordinate x.
    /// @param[in] x Horizontal spatial coordinate in meters.
    /// @return Upward unit normal vector.
    [[nodiscard]] math::Vec2 getNormal(double x) const noexcept;

    /// @brief Evaluates unit forward surface tangent vector at coordinate x.
    /// @param[in] x Horizontal spatial coordinate in meters.
    /// @return Forward unit tangent vector.
    [[nodiscard]] math::Vec2 getTangent(double x) const noexcept;

    /// @brief Queries physical surface material at coordinate x.
    /// @param[in] x Horizontal spatial coordinate in meters.
    /// @return SurfaceMaterial active at coordinate x.
    [[nodiscard]] SurfaceMaterial getMaterial(double x) const noexcept;

    /// @brief Evaluates full surface geometry and material sample at coordinate x.
    /// @param[in] x Horizontal spatial coordinate in meters.
    /// @return SurfaceSample1D containing height, slope, normal, tangent, and material.
    [[nodiscard]] SurfaceSample1D sampleSurface(double x) const noexcept;

    /// @brief Computes terrain penetration depth for a 2D point.
    /// @details Positive if point is underground (below surface), negative if airborne (above surface).
    /// @param[in] point World coordinates of test point.
    /// @return Signed vertical penetration depth in meters (h(point.x) - point.y).
    [[nodiscard]] double getPenetration(const math::Vec2& point) const noexcept;

    /// @brief Tests whether a 2D point is submerged below the terrain surface.
    /// @param[in] point World coordinates of test point.
    /// @return True if point elevation is strictly less than terrain surface height.
    [[nodiscard]] bool isUnderground(const math::Vec2& point) const noexcept;

    /// @brief Extracts discrete horizontal platforms from heightfield profile.
    /// @param[in] config Platform extraction parameters.
    /// @return Vector of extracted HeightfieldPlatform descriptors.
    [[nodiscard]] std::vector<HeightfieldPlatform> extractPlatforms(const PlatformExtractionConfig& config) const;

    /// @brief Extracts discrete platforms converted directly into SideViewPlatform structs.
    /// @param[in] config Platform extraction parameters.
    /// @return Vector of dynamics-layer SideViewPlatform descriptors.
    [[nodiscard]] std::vector<dynamics::SideViewPlatform> extractSideViewPlatforms(
        const PlatformExtractionConfig& config) const;

    /// @brief Extracts platforms and populates them directly into SideViewCarDynamics coordinator.
    /// @param[in,out] car Target side-view platform car dynamics coordinator.
    /// @param[in] config Platform extraction parameters.
    /// @param[in] clearExisting True to clear prior platforms registered in car coordinator.
    void populateSideViewCar(
        dynamics::SideViewCarDynamics& car, const PlatformExtractionConfig& config, bool clearExisting = true) const;

private:
    [[nodiscard]] std::pair<double, double> sampleDiscrete(double x) const noexcept;

    double mBaseHeight { 0.0 };
    double mVerticalScale { 1.0 };
    double mHorizontalScale { 1.0 };
    double mHorizontalOffset { 0.0 };
    bool mEnableNoise { false };
    HeightfieldInterp mInterpolation { HeightfieldInterp::Linear };
    HeightfieldBoundary mBoundaryMode { HeightfieldBoundary::Clamp };
    SurfaceMaterial mDefaultMaterial {};
    FractalNoise mNoise {};
    std::vector<HeightSample1D> mSamples {};
    std::vector<MaterialZone> mMaterialZones {};
};

} // namespace vehicle_sim::terrain
