#pragma once

#include <cmath>
#include <cstddef>
#include <vector>

#include "libs/VehicleSimCore/math/Vec2.h"
#include "libs/VehicleSimCore/terrain/SurfaceMaterial.h"

namespace vehicle_sim::terrain {

struct StratifiedSoilConfig;
struct StratifiedSinkageResult;

/// @struct RutStamp2D
/// @brief Wheel or track contact footprint description for terrain rut deformation.
struct RutStamp2D {
    math::Vec2 position { 0.0, 0.0 }; ///< Contact center in world coordinates in meters.
    double heading { 0.0 }; ///< Heading direction angle in radians.
    double width { 0.30 }; ///< Contact patch width b in meters.
    double length { 0.40 }; ///< Contact patch length L in meters.
    double diameter { 0.70 }; ///< Wheel rolling outer diameter D in meters.
    double normalForce { 0.0 }; ///< Normal contact load Fz in Newtons.
    math::Vec2 velocity { 0.0, 0.0 }; ///< Contact patch linear velocity vector in m/s.
};

/// @struct SoilSinkageResult
/// @brief Analytical terramechanics evaluation output for a contact footprint.
struct SoilSinkageResult {
    double sinkage { 0.0 }; ///< Total penetration depth z in meters.
    double plasticSinkage { 0.0 }; ///< Permanent plastic depression depth z_p in meters.
    double compactionDrag { 0.0 }; ///< Bekker soil compaction drag magnitude Rc in Newtons.
    double bulldozeDrag { 0.0 }; ///< Hegazy-Bekker bulldozing drag magnitude Rb in Newtons.
    math::Vec2 dragForce { 0.0, 0.0 }; ///< Planar drag force vector opposing velocity in Newtons.
    double contactPressure { 0.0 }; ///< Ground contact pressure p in N/m^2.
    double bermHeight { 0.0 }; ///< Peak extruded lateral berm ridge elevation in meters.
};

/// @class BekkerWongSolver
/// @brief Pure analytical solver for Bekker-Wong soil sinkage and terramechanics drag forces.
class BekkerWongSolver {
public:
    /// @brief Computes effective soil stiffness modulus k_eq.
    /// @param[in] width Contact width b in meters.
    /// @param[in] soil Soil mechanics configuration.
    /// @return Equivalent stiffness k_eq in N/m^(n+2).
    [[nodiscard]] static double computeModulus(double width, const SoilMechanicsConfig& soil) noexcept;

    /// @brief Evaluates Bekker sinkage depth z from contact pressure.
    /// @param[in] pressure Ground contact pressure p in N/m^2.
    /// @param[in] width Contact width b in meters.
    /// @param[in] soil Soil mechanics configuration.
    /// @return Sinkage depth z in meters clamped to maxPlasticDepth.
    [[nodiscard]] static double computeSinkage(double pressure, double width, const SoilMechanicsConfig& soil) noexcept;

    /// @brief Evaluates Bekker compaction drag resistance force Rc.
    /// @param[in] sinkage Penetration depth z in meters.
    /// @param[in] width Contact width b in meters.
    /// @param[in] soil Soil mechanics configuration.
    /// @return Compaction resistance force in Newtons.
    [[nodiscard]] static double computeCompactionDrag(
        double sinkage, double width, const SoilMechanicsConfig& soil) noexcept;

    /// @brief Evaluates Hegazy-Bekker passive earth pressure bulldozing drag Rb.
    /// @param[in] sinkage Penetration depth z in meters.
    /// @param[in] width Contact width b in meters.
    /// @param[in] soil Soil mechanics configuration.
    /// @return Bulldozing resistance force in Newtons.
    [[nodiscard]] static double computeBulldozeDrag(
        double sinkage, double width, const SoilMechanicsConfig& soil) noexcept;

    /// @brief Evaluates full terramechanics forces and sinkage for a wheel stamp.
    /// @param[in] stamp Wheel or track contact footprint parameters.
    /// @param[in] soil Soil mechanics configuration.
    /// @return SoilSinkageResult with all forces and geometric depths.
    [[nodiscard]] static SoilSinkageResult evaluateStamp(
        const RutStamp2D& stamp, const SoilMechanicsConfig& soil) noexcept;
};

/// @class DeformableTerrainLayer
/// @brief Discrete 2D delta-elevation grid tracking plastic ruts, berms, and relaxation.
class DeformableTerrainLayer {
public:
    /// @brief Default constructor.
    DeformableTerrainLayer() noexcept = default;

    /// @brief Initializes deformable layer dimensions matching parent SurfaceMap2D.
    /// @param[in] origin Minimum corner coordinates in world meters.
    /// @param[in] cellSize Grid cell dimensions [dx, dy] in meters.
    /// @param[in] width Column count along X axis.
    /// @param[in] height Row count along Y axis.
    void initGrid(const math::Vec2& origin, const math::Vec2& cellSize, std::size_t width, std::size_t height);

    /// @brief Applies a wheel or track rut stamp, mutating delta elevation values.
    /// @param[in] stamp Wheel footprint geometry, load, and kinematics.
    /// @param[in] soil Soil mechanics material properties.
    /// @return SoilSinkageResult containing forces and sinkage depth.
    SoilSinkageResult applyRutStamp(const RutStamp2D& stamp, const SoilMechanicsConfig& soil) noexcept;

    /// @brief Advances viscoelastic relaxation decay over timestep dt.
    /// @param[in] dt Timestep duration in seconds.
    /// @param[in] soil Active soil mechanics parameters.
    void updateRelaxation(double dt, const SoilMechanicsConfig& soil) noexcept;

    /// @brief Evaluates total rut elevation delta (plastic + elastic) at world (x, y).
    /// @param[in] x World X coordinate in meters.
    /// @param[in] y World Y coordinate in meters.
    /// @return Elevation offset in meters (negative in ruts, positive on berms).
    [[nodiscard]] double getRutDelta(double x, double y) const noexcept;

    /// @brief Clears all accumulated ruts and resets layer to flat zero.
    void clearRuts() noexcept;

    /// @brief Queries whether grid layer is initialized.
    /// @return True if dimensions are non-zero.
    [[nodiscard]] bool isInitialized() const noexcept;

    /// @brief Gets maximum depth among all existing wheel ruts.
    /// @return Maximum rut depression in meters (positive value).
    [[nodiscard]] double getMaxRutDepth() const noexcept;

    /// @brief Gets grid column count along X axis.
    /// @return Number of columns.
    [[nodiscard]] std::size_t getGridWidth() const noexcept;

    /// @brief Gets grid row count along Y axis.
    /// @return Number of rows.
    [[nodiscard]] std::size_t getGridHeight() const noexcept;

    /// @brief Gets discrete plastic elevation offset at grid vertex index.
    /// @param[in] index Linear index in [0, width * height - 1].
    /// @return Plastic elevation delta in meters.
    [[nodiscard]] double getPlasticDelta(std::size_t index) const noexcept;

    /// @brief Gets discrete elastic elevation offset at grid vertex index.
    /// @param[in] index Linear index in [0, width * height - 1].
    /// @return Elastic elevation delta in meters.
    [[nodiscard]] double getElasticDelta(std::size_t index) const noexcept;

    /// @brief Sets baseline uniform upper stratum depth.
    /// @param[in] depth Upper stratum layer thickness in meters.
    void setTopStratumDepth(double depth) noexcept;

    /// @brief Configures spatially varying stratum depth grid.
    /// @param[in] depths Flattened row-major layer depths in meters.
    void setTopStratumGrid(const std::vector<double>& depths);

    /// @brief Queries upper stratum depth at world coordinates (x, y).
    /// @param[in] x World X coordinate in meters.
    /// @param[in] y World Y coordinate in meters.
    /// @return Stratum thickness d_top in meters.
    [[nodiscard]] double getStratumDepth(double x, double y) const noexcept;

    /// @brief Applies a stratified rut stamp, mutating delta elevations.
    /// @param[in] stamp Wheel footprint geometry, load, and kinematics.
    /// @param[in] config Stratified soil properties.
    /// @return StratifiedSinkageResult with partitioned forces and depths.
    StratifiedSinkageResult applyStratifiedRut(
        const RutStamp2D& stamp, const StratifiedSoilConfig& config) noexcept;

private:
    [[nodiscard]] double sampleBilinear(const std::vector<double>& data, double x, double y) const noexcept;

    math::Vec2 mOrigin { 0.0, 0.0 };
    math::Vec2 mCellSize { 1.0, 1.0 };
    std::size_t mWidth { 0U };
    std::size_t mHeight { 0U };

    std::vector<double> mPlasticDelta {}; ///< Permanent plastic elevation delta (negative = depression).
    std::vector<double> mElasticDelta {}; ///< Transient viscoelastic elevation delta.
    std::vector<double> mHardening {}; ///< Accumulated plastic compaction depth.
    double mBaseStratumDepth { 0.15 }; ///< Uniform baseline stratum thickness in meters.
    std::vector<double> mStratumDepth {}; ///< Spatially varying stratum depth array.
};

} // namespace vehicle_sim::terrain
