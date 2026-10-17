#pragma once

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

#include "libs/VehicleSimCore/dynamics/ArcadeCarDynamics.h"
#include "libs/VehicleSimCore/math/Vec2.h"
#include "libs/VehicleSimCore/physics/RigidBody2D.h"
#include "libs/VehicleSimCore/terrain/DeformableTerrain.h"
#include "libs/VehicleSimCore/terrain/FractalNoise.h"
#include "libs/VehicleSimCore/terrain/SurfaceMaterial.h"

namespace vehicle_sim::terrain {

/// @enum ElevationInterp2D
/// @brief Spatial interpolation algorithm across discrete 2D elevation grid samples.
enum class ElevationInterp2D : std::uint8_t {
    Nearest, ///< Nearest-neighbor discrete step interpolation.
    Bilinear, ///< Bilinear C0 continuous interpolation with piecewise linear gradients.
    Bicubic ///< Bicubic Hermite / Catmull-Rom C1 continuous spline with smooth gradients.
};

/// @enum ElevationBoundary2D
/// @brief Out-of-bounds coordinate handling policy for 2D elevation grid.
enum class ElevationBoundary2D : std::uint8_t {
    Clamp, ///< Clamps evaluation coordinates to nearest grid boundary.
    Repeat, ///< Periodic wrapping (toroidal repeat) for seamless looping worlds.
    Zero ///< Returns zero elevation outside grid boundaries.
};

/// @enum ZoneShape2D
/// @brief Geometric primitive shape for 2D surface material zones.
enum class ZoneShape2D : std::uint8_t {
    Box, ///< Axis-aligned bounding box defined by min and max corners.
    Circle ///< Radial circular disk defined by center and radius.
};

/// @struct MaterialZone2D
/// @brief 2D spatial region overriding default surface physical parameters.
struct MaterialZone2D {
    ZoneShape2D shape { ZoneShape2D::Box };
    math::Vec2 min { 0.0, 0.0 }; ///< Box min corner in meters [minX, minY].
    math::Vec2 max { 0.0, 0.0 }; ///< Box max corner in meters [maxX, maxY].
    math::Vec2 center { 0.0, 0.0 }; ///< Circle center coordinates in meters [cx, cy].
    double radius { 0.0 }; ///< Circle radius in meters.
    SurfaceMaterial material {}; ///< Surface material properties applied within region.
};

/// @struct SurfaceNormal3D
/// @brief 3D unit surface normal vector in Cartesian coordinates.
/// @details Represents normal vector (x, y, z) pointing upwards away from terrain,
/// where z > 0 and x^2 + y^2 + z^2 == 1.
struct SurfaceNormal3D {
    double x { 0.0 }; ///< Normal vector X component.
    double y { 0.0 }; ///< Normal vector Y component.
    double z { 1.0 }; ///< Normal vector Z vertical component.
};

/// @struct SurfaceSample2D
/// @brief Complete geometric and physical surface sample at world coordinates (x, y).
struct SurfaceSample2D {
    double elevation { 0.0 }; ///< Surface elevation z = h(x, y) in meters.
    math::Vec2 gradient { 0.0, 0.0 }; ///< First-order spatial gradient vector (dh/dx, dh/dy).
    double slopeAngle { 0.0 }; ///< Surface inclination angle in radians in range [0, pi/2).
    math::Vec2 downhill { 0.0, 0.0 }; ///< Unit planar downhill steepest descent vector.
    SurfaceNormal3D normal { 0.0, 0.0, 1.0 }; ///< 3D upward unit surface normal vector.
    SurfaceMaterial material {}; ///< Active surface physical parameters.
};

/// @class SurfaceMap2D
/// @brief 2D top-down surface and elevation map supporting discrete grids, noise, and vehicle dynamics.
/// @details Combines discrete regular elevation grids (Nearest, Bilinear, Bicubic), procedural multi-octave
/// fractal noise, analytical 2D spatial gradients, upward 3D surface normals, downhill slope forces,
/// surface material zones (Box, Circle), and direct integration with top-down vehicle dynamics coordinators.
///
/// ```
///  ASCII Diagram: 2D Top-Down Elevation Map & Surface Zones
///
///    Y (North)
///     ^
///     |       +------------------------------------+
///     |       | Zone B: Mud Pit (Circle)           |
///     |       |         (cx, cy, r)                |
///     |       |          .-'''--.                  |
///     |       |        .'        '.                |
///     |       |       /   *        \               |
///     |       |        '.        .'                |
///     |       |          '-...--'                  |
///     |       |                                    |
///     |       |       Gradient [dh/dx, dh/dy]      |
///     |       |          \                         |
///     |       |           v Downhill Force         |
///     |       |  +--------------------+            |
///     |       |  | Zone A: Ice (Box)  |            |
///     |       |  +--------------------+            |
///     +-------+------------------------------------+----> X (East)
///             (originX, originY)
/// ```
///
/// ```mermaid
/// graph TD
///     subgraph EvaluationPipeline ["Evaluation Pipeline"]
///         WorldPos["World Position (x, y)"] --> GridNode["2D Grid Interpolation (Nearest / Bilinear / Bicubic)"]
///         WorldPos --> NoiseNode["2D Fractal Noise Layer (fBm / Billow / Ridged)"]
///         GridNode --> SumNode["Combined Elevation & Gradient Vector (dh/dx, dh/dy)"]
///         NoiseNode --> SumNode
///         SumNode --> NormalNode["3D Upward Surface Normal & Downhill Vector"]
///         WorldPos --> ZoneNode["2D Surface Material Zones (Box / Circle)"]
///         NormalNode --> SampleOut["SurfaceSample2D Output"]
///         ZoneNode --> SampleOut
///     end
///     subgraph VehicleInteraction ["Vehicle Interaction"]
///         SampleOut --> SlopeForceNode["Planar Downhill Slope Force (m * g * sin(theta))"]
///         SampleOut --> GripNode["Tire Grip & Rolling Drag Modifier Application"]
///         SlopeForceNode --> RigidBody["Physics RigidBody2D"]
///         GripNode --> ArcadeCar["ArcadeCarDynamics"]
///     end
/// ```
class SurfaceMap2D {
public:
    /// @brief Default constructor with baseline flat tarmac profile.
    SurfaceMap2D() noexcept = default;

    /// @brief Sets baseline uniform vertical elevation.
    /// @param[in] baseHeight Base elevation in meters.
    void setBaseHeight(double baseHeight) noexcept;

    /// @brief Gets baseline uniform vertical elevation.
    /// @return Base elevation in meters.
    [[nodiscard]] double getBaseHeight() const noexcept;

    /// @brief Configures regular 2D discrete elevation grid.
    /// @param[in] origin World coordinates of grid minimum corner [minX, minY] in meters.
    /// @param[in] cellSize Spatial spacing of cells [dx, dy] in meters.
    /// @param[in] width Number of columns along X axis.
    /// @param[in] height Number of rows along Y axis.
    /// @param[in] elevations Row-major flattened elevation values (size must equal width * height).
    void setGrid(const math::Vec2& origin, const math::Vec2& cellSize, std::size_t width, std::size_t height,
        const std::vector<double>& elevations);

    /// @brief Modifies a single discrete elevation grid vertex.
    /// @param[in] xIndex Column index in [0, width - 1].
    /// @param[in] yIndex Row index in [0, height - 1].
    /// @param[in] elevation New elevation value in meters.
    void setGridElevation(std::size_t xIndex, std::size_t yIndex, double elevation) noexcept;

    /// @brief Retrieves a single discrete elevation grid vertex.
    /// @param[in] xIndex Column index in [0, width - 1].
    /// @param[in] yIndex Row index in [0, height - 1].
    /// @return Elevation value in meters (0.0 if out of bounds).
    [[nodiscard]] double getGridElevation(std::size_t xIndex, std::size_t yIndex) const noexcept;

    /// @brief Clears discrete elevation grid data.
    void clearGrid() noexcept;

    /// @brief Queries whether discrete elevation grid is configured.
    /// @return True if grid has non-zero dimensions and data.
    [[nodiscard]] bool hasGrid() const noexcept;

    /// @brief Gets number of grid columns along X axis.
    /// @return Grid column count.
    [[nodiscard]] std::size_t getGridWidth() const noexcept;

    /// @brief Gets number of grid rows along Y axis.
    /// @return Grid row count.
    [[nodiscard]] std::size_t getGridHeight() const noexcept;

    /// @brief Gets grid world origin coordinate.
    /// @return World position of grid minimum corner.
    [[nodiscard]] math::Vec2 getGridOrigin() const noexcept;

    /// @brief Gets grid cell dimensions.
    /// @return Cell spatial spacing [dx, dy] in meters.
    [[nodiscard]] math::Vec2 getCellSize() const noexcept;

    /// @brief Sets discrete grid spatial interpolation algorithm.
    /// @param[in] interp Target interpolation method (Nearest, Bilinear, Bicubic).
    void setInterpolation(ElevationInterp2D interp) noexcept;

    /// @brief Gets discrete grid spatial interpolation algorithm.
    /// @return Current interpolation mode.
    [[nodiscard]] ElevationInterp2D getInterpolation() const noexcept;

    /// @brief Sets out-of-bounds grid boundary handling mode.
    /// @param[in] boundary Target boundary policy (Clamp, Repeat, Zero).
    void setBoundaryMode(ElevationBoundary2D boundary) noexcept;

    /// @brief Gets out-of-bounds grid boundary handling mode.
    /// @return Current boundary policy.
    [[nodiscard]] ElevationBoundary2D getBoundaryMode() const noexcept;

    /// @brief Sets procedural noise scaling and offset parameters.
    /// @param[in] verticalScale Amplitude multiplier for procedural elevation in meters.
    /// @param[in] horizontalScale Spatial frequency multiplier along [X, Y] axes.
    /// @param[in] offset Spatial offset translation along [X, Y] axes in meters.
    void setNoiseTransform(double verticalScale, const math::Vec2& horizontalScale, const math::Vec2& offset) noexcept;

    /// @brief Enables or disables procedural fractal noise evaluation.
    /// @param[in] enable True to enable procedural noise contribution.
    void enableNoise(bool enable) noexcept;

    /// @brief Queries whether procedural fractal noise is enabled.
    /// @return True if procedural noise is active.
    [[nodiscard]] bool isNoiseEnabled() const noexcept;

    /// @brief Sets procedural fractal noise synthesizer configuration.
    /// @param[in] config Fractal noise configuration parameters.
    void setNoiseConfig(const FractalConfig& config) noexcept;

    /// @brief Gets active procedural fractal noise synthesizer.
    /// @return Const reference to internal FractalNoise object.
    [[nodiscard]] const FractalNoise& getNoise() const noexcept;

    /// @brief Sets default surface physical material.
    /// @param[in] material Material properties used outside defined zones.
    void setDefaultMaterial(const SurfaceMaterial& material) noexcept;

    /// @brief Gets default surface physical material.
    /// @return Current default surface material.
    [[nodiscard]] const SurfaceMaterial& getDefaultMaterial() const noexcept;

    /// @brief Adds an axis-aligned box override for surface material.
    /// @param[in] min Minimum corner coordinates in meters [minX, minY].
    /// @param[in] max Maximum corner coordinates in meters [maxX, maxY].
    /// @param[in] material Surface physical material properties.
    void addBoxZone(const math::Vec2& min, const math::Vec2& max, const SurfaceMaterial& material);

    /// @brief Adds an axis-aligned box override using a material preset.
    /// @param[in] min Minimum corner coordinates in meters [minX, minY].
    /// @param[in] max Maximum corner coordinates in meters [maxX, maxY].
    /// @param[in] materialType Target material preset type.
    void addBoxZone(const math::Vec2& min, const math::Vec2& max, MaterialType materialType);

    /// @brief Adds a circular radial override for surface material.
    /// @param[in] center Center coordinates in meters [cx, cy].
    /// @param[in] radius Circular radius in meters.
    /// @param[in] material Surface physical material properties.
    void addCircleZone(const math::Vec2& center, double radius, const SurfaceMaterial& material);

    /// @brief Adds a circular radial override using a material preset.
    /// @param[in] center Center coordinates in meters [cx, cy].
    /// @param[in] radius Circular radius in meters.
    /// @param[in] materialType Target material preset type.
    void addCircleZone(const math::Vec2& center, double radius, MaterialType materialType);

    /// @brief Stamps a circular material patch into the surface map.
    /// @param[in] center World coordinates of circle center in meters.
    /// @param[in] radius Circular footprint radius in meters.
    /// @param[in] material Material properties applied within circle.
    void stampMaterialDisc(
        const math::Vec2& center, double radius, const SurfaceMaterial& material);

    /// @brief Removes all registered material zone overrides.
    void clearMaterialZones() noexcept;

    /// @brief Gets registered material zone count.
    /// @return Number of registered material zones.
    [[nodiscard]] std::size_t getMaterialZoneCount() const noexcept;

    /// @brief Queries physical surface material at coordinates (x, y).
    /// @param[in] x Horizontal spatial coordinate in meters.
    /// @param[in] y Vertical spatial coordinate in meters.
    /// @return SurfaceMaterial active at specified coordinates.
    [[nodiscard]] SurfaceMaterial getMaterial(double x, double y) const noexcept;

    /// @brief Queries physical surface material at vector position.
    /// @param[in] point World coordinates in meters.
    /// @return SurfaceMaterial active at point.
    [[nodiscard]] SurfaceMaterial getMaterial(const math::Vec2& point) const noexcept;

    /// @brief Evaluates surface elevation z = h(x, y) at coordinates (x, y).
    /// @param[in] x Horizontal spatial coordinate in meters.
    /// @param[in] y Vertical spatial coordinate in meters.
    /// @return Surface elevation in meters.
    [[nodiscard]] double getHeight(double x, double y) const noexcept;

    /// @brief Evaluates surface elevation z = h(point) at vector position.
    /// @param[in] point World coordinates in meters.
    /// @return Surface elevation in meters.
    [[nodiscard]] double getHeight(const math::Vec2& point) const noexcept;

    /// @brief Evaluates first-order spatial gradient vector (dh/dx, dh/dy) at (x, y).
    /// @param[in] x Horizontal spatial coordinate in meters.
    /// @param[in] y Vertical spatial coordinate in meters.
    /// @return 2D gradient vector in meters per meter.
    [[nodiscard]] math::Vec2 getGradient(double x, double y) const noexcept;

    /// @brief Evaluates first-order spatial gradient vector at vector position.
    /// @param[in] point World coordinates in meters.
    /// @return 2D gradient vector in meters per meter.
    [[nodiscard]] math::Vec2 getGradient(const math::Vec2& point) const noexcept;

    /// @brief Evaluates terrain slope inclination angle at coordinates (x, y).
    /// @param[in] x Horizontal spatial coordinate in meters.
    /// @param[in] y Vertical spatial coordinate in meters.
    /// @return Inclination angle in radians in range [0, pi/2).
    [[nodiscard]] double getSlopeAngle(double x, double y) const noexcept;

    /// @brief Evaluates terrain slope inclination angle at vector position.
    /// @param[in] point World coordinates in meters.
    /// @return Inclination angle in radians in range [0, pi/2).
    [[nodiscard]] double getSlopeAngle(const math::Vec2& point) const noexcept;

    /// @brief Evaluates unit planar downhill direction vector at coordinates (x, y).
    /// @param[in] x Horizontal spatial coordinate in meters.
    /// @param[in] y Vertical spatial coordinate in meters.
    /// @return Unit downhill direction vector (zero if flat).
    [[nodiscard]] math::Vec2 getDownhill(double x, double y) const noexcept;

    /// @brief Evaluates unit planar downhill direction vector at vector position.
    /// @param[in] point World coordinates in meters.
    /// @return Unit downhill direction vector (zero if flat).
    [[nodiscard]] math::Vec2 getDownhill(const math::Vec2& point) const noexcept;

    /// @brief Evaluates 3D upward unit surface normal vector at coordinates (x, y).
    /// @param[in] x Horizontal spatial coordinate in meters.
    /// @param[in] y Vertical spatial coordinate in meters.
    /// @return SurfaceNormal3D with unit length.
    [[nodiscard]] SurfaceNormal3D getNormal3D(double x, double y) const noexcept;

    /// @brief Evaluates 3D upward unit surface normal vector at vector position.
    /// @param[in] point World coordinates in meters.
    /// @return SurfaceNormal3D with unit length.
    [[nodiscard]] SurfaceNormal3D getNormal3D(const math::Vec2& point) const noexcept;

    /// @brief Evaluates full surface geometry and material sample at (x, y).
    /// @param[in] x Horizontal spatial coordinate in meters.
    /// @param[in] y Vertical spatial coordinate in meters.
    /// @return SurfaceSample2D containing height, gradient, slope, normal, and material.
    [[nodiscard]] SurfaceSample2D sampleSurface(double x, double y) const noexcept;

    /// @brief Evaluates full surface geometry and material sample at vector position.
    /// @param[in] point World coordinates in meters.
    /// @return SurfaceSample2D containing height, gradient, slope, normal, and material.
    [[nodiscard]] SurfaceSample2D sampleSurface(const math::Vec2& point) const noexcept;

    /// @brief Tests whether world position is located within a liquid surface medium.
    /// @param[in] point World coordinates in meters.
    /// @return True if active material is flagged as liquid.
    [[nodiscard]] bool isLiquidAt(const math::Vec2& point) const noexcept;

    /// @brief Computes planar downhill slope force acting on a rigid body.
    /// @details Computes F = -m * g * (gradient / sqrt(1 + |gradient|^2)).
    /// @param[in] body Target physics rigid body.
    /// @param[in] gravity Gravitational acceleration magnitude in m/s^2 (default 9.81).
    /// @return Planar force vector in Newtons.
    [[nodiscard]] math::Vec2 computeSlopeForce(const physics::RigidBody2D& body, double gravity = 9.81) const noexcept;

    /// @brief Applies planar downhill slope force directly to a rigid body.
    /// @param[in,out] body Target physics rigid body to receive gravity force.
    /// @param[in] gravity Gravitational acceleration magnitude in m/s^2 (default 9.81).
    void applySlopeForce(physics::RigidBody2D& body, double gravity = 9.81) const noexcept;

    /// @brief Computes liquid viscous drag retarding force acting on a submerged body.
    /// @param[in] body Target physics rigid body.
    /// @param[in] dragFactor Additional viscous scale factor.
    /// @return Retarding force vector in Newtons (zero if not in liquid).
    [[nodiscard]] math::Vec2 computeLiquidDrag(
        const physics::RigidBody2D& body, double dragFactor = 1.0) const noexcept;

    /// @brief Samples terrain and updates arcade car dynamics tire grip and rolling drag.
    /// @details Evaluates ground contact under each individual wheel, updating individual
    /// wheel grip multipliers as well as global chassis surface drag.
    /// @param[in,out] car Target arcade car dynamics coordinator.
    /// @param[in] body Associated vehicle rigid body in world space.
    void applySurfaceToCar(dynamics::ArcadeCarDynamics& car, const physics::RigidBody2D& body) const noexcept;

    /// @brief Enables or disables deformable soil rutting layer.
    /// @param[in] enable True to enable deformable soil rutting.
    void enableDeformableLayer(bool enable) noexcept;

    /// @brief Queries whether deformable soil rutting is enabled.
    /// @return True if deformable layer is active.
    [[nodiscard]] bool isDeformableEnabled() const noexcept;

    /// @brief Configures deformable soil rutting grid dimensions.
    /// @param[in] origin Minimum corner coordinates in world meters.
    /// @param[in] cellSize Cell dimensions [dx, dy] in meters.
    /// @param[in] width Number of columns.
    /// @param[in] height Number of rows.
    void initDeformableLayer(
        const math::Vec2& origin, const math::Vec2& cellSize, std::size_t width, std::size_t height);

    /// @brief Applies a wheel or track rut stamp and mutates surface elevations.
    /// @param[in] stamp Wheel footprint geometry, load, and kinematics.
    /// @return SoilSinkageResult containing forces and sinkage depth.
    SoilSinkageResult applyRutStamp(const RutStamp2D& stamp) noexcept;

    /// @brief Advances viscoelastic soil recovery over timestep dt.
    /// @param[in] dt Elapsed time in seconds.
    void updateRutRecovery(double dt) noexcept;

    /// @brief Clears all accumulated wheel ruts from the deformable layer.
    void clearRuts() noexcept;

    /// @brief Evaluates total surface elevation including deformable rutting.
    /// @param[in] x World X coordinate in meters.
    /// @param[in] y World Y coordinate in meters.
    /// @return Surface elevation in meters.
    [[nodiscard]] double getDeformedHeight(double x, double y) const noexcept;

    /// @brief Evaluates total surface elevation including deformable rutting.
    /// @param[in] point World coordinates in meters.
    /// @return Surface elevation in meters.
    [[nodiscard]] double getDeformedHeight(const math::Vec2& point) const noexcept;

    /// @brief Evaluates rut elevation offset (negative in ruts, positive on berms).
    /// @param[in] x World X coordinate in meters.
    /// @param[in] y World Y coordinate in meters.
    /// @return Rut delta in meters.
    [[nodiscard]] double getRutDelta(double x, double y) const noexcept;

    /// @brief Gets maximum depth among all accumulated wheel ruts.
    /// @return Maximum rut depression in meters (positive value).
    [[nodiscard]] double getMaxRutDepth() const noexcept;

    /// @brief Gets const reference to underlying deformable terrain layer.
    /// @return Const reference to internal DeformableTerrainLayer.
    [[nodiscard]] const DeformableTerrainLayer& getDeformableLayer() const noexcept;

    /// @brief Gets mutable reference to underlying deformable terrain layer.
    /// @return Mutable reference to internal DeformableTerrainLayer.
    [[nodiscard]] DeformableTerrainLayer& getDeformableLayer() noexcept;

private:
    [[nodiscard]] std::pair<double, math::Vec2> sampleGrid(double x, double y) const noexcept;
    [[nodiscard]] double getGridSample(int ix, int iy) const noexcept;

    double mBaseHeight { 0.0 };
    double mVerticalScale { 1.0 };
    math::Vec2 mHorizontalScale { 1.0, 1.0 };
    math::Vec2 mNoiseOffset { 0.0, 0.0 };
    bool mEnableNoise { false };
    ElevationInterp2D mInterpolation { ElevationInterp2D::Bilinear };
    ElevationBoundary2D mBoundaryMode { ElevationBoundary2D::Clamp };

    math::Vec2 mGridOrigin { 0.0, 0.0 };
    math::Vec2 mCellSize { 1.0, 1.0 };
    std::size_t mGridWidth { 0U };
    std::size_t mGridHeight { 0U };
    std::vector<double> mGridData {};

    SurfaceMaterial mDefaultMaterial {};
    FractalNoise mNoise {};
    std::vector<MaterialZone2D> mMaterialZones {};

    bool mEnableDeformable { false };
    DeformableTerrainLayer mDeformableLayer {};
};

/// @brief Type alias for 2D top-down surface and elevation map.
using ElevationMap2D = SurfaceMap2D;

} // namespace vehicle_sim::terrain
