#pragma once

#include <cmath>
#include <cstddef>
#include <string_view>

#include "libs/VehicleSimCore/math/Vec2.h"
#include "libs/VehicleSimCore/terrain/DeformableTerrain.h"
#include "libs/VehicleSimCore/terrain/SurfaceMaterial.h"

namespace vehicle_sim::terrain {

/// @struct StratifiedLayerConfig
/// @brief Terramechanics properties for a single geological soil stratum.
struct StratifiedLayerConfig {
    SoilMechanicsConfig soil {}; ///< Bekker-Wong soil mechanics properties.
    double layerDepth { 0.15 }; ///< Stratum layer thickness in meters.
    double bearingCapacity { 500000.0 }; ///< Ultimate bearing failure limit in N/m^2.
    std::string_view name { "LooseTopsoil" }; ///< Diagnostic stratum identifier.
};

/// @struct StratifiedSoilConfig
/// @brief Multi-layer geological stratification configuration.
/// @details Defines an upper compressible stratum resting on a stiff substrate.
struct StratifiedSoilConfig {
    StratifiedLayerConfig topLayer {}; ///< Upper compressible soil stratum (mud, snow, sand).
    StratifiedLayerConfig substrate {}; ///< Stiff base stratum (bedrock, hardpan, asphalt).
    double transitionZone { 0.02 }; ///< Hermite blending half-width delta_s in meters.
    bool enableStratification { true }; ///< Flag toggling multi-layer vs homogeneous model.
};

/// @struct StratifiedSinkageResult
/// @brief Terramechanics evaluation output for stratified soil contact.
struct StratifiedSinkageResult {
    double totalSinkage { 0.0 }; ///< Total penetration depth z in meters.
    double topLayerSinkage { 0.0 }; ///< Penetration depth within upper layer z1 in meters.
    double substrateSinkage { 0.0 }; ///< Penetration depth into substrate z2 in meters.
    double effectiveModulus { 0.0 }; ///< Blended stiffness modulus k_eff in N/m^(n+2).
    double compactionDrag { 0.0 }; ///< Total stratified compaction drag Rc in Newtons.
    double bulldozeDrag { 0.0 }; ///< Bounded upper-layer bulldoze drag Rb in Newtons.
    math::Vec2 dragForce { 0.0, 0.0 }; ///< Net horizontal drag force vector in Newtons.
    bool reachedSubstrate { false }; ///< True if wheel punched through top stratum.
};

/// @class StratifiedSoilSolver
/// @brief Analytical terramechanics solver for multi-layer stratified ground profiles.
class StratifiedSoilSolver {
public:
    StratifiedSoilSolver() = delete;

    /// @brief Computes blended effective stiffness modulus at depth z.
    /// @param[in] depth Penetration depth z in meters.
    /// @param[in] width Contact width b in meters.
    /// @param[in] config Stratified soil configuration.
    /// @return Effective soil stiffness modulus k_eff in N/m^(n+2).
    [[nodiscard]] static double computeModulus(
        double depth, double width, const StratifiedSoilConfig& config) noexcept;

    /// @brief Evaluates stratified sinkage depth z from contact pressure.
    /// @param[in] pressure Normal ground contact pressure p in N/m^2.
    /// @param[in] width Contact width b in meters.
    /// @param[in] config Stratified soil configuration.
    /// @return StratifiedSinkageResult containing depths and substrate status.
    [[nodiscard]] static StratifiedSinkageResult computeSinkage(
        double pressure, double width, const StratifiedSoilConfig& config) noexcept;

    /// @brief Evaluates stratified soil compaction drag resistance force Rc.
    /// @param[in] sinkage Penetration depth z in meters.
    /// @param[in] width Contact width b in meters.
    /// @param[in] config Stratified soil configuration.
    /// @return Compaction resistance force in Newtons.
    [[nodiscard]] static double computeCompactionDrag(
        double sinkage, double width, const StratifiedSoilConfig& config) noexcept;

    /// @brief Evaluates upper-stratum bounded Hegazy-Bekker bulldozing drag Rb.
    /// @param[in] sinkage Penetration depth z in meters.
    /// @param[in] width Contact width b in meters.
    /// @param[in] config Stratified soil configuration.
    /// @return Bulldozing drag resistance force in Newtons.
    [[nodiscard]] static double computeBulldozeDrag(
        double sinkage, double width, const StratifiedSoilConfig& config) noexcept;

    /// @brief Evaluates full stratified terramechanics forces for a contact stamp.
    /// @param[in] stamp Wheel or track contact footprint parameters.
    /// @param[in] config Stratified soil configuration.
    /// @return Complete StratifiedSinkageResult with all forces and depths.
    [[nodiscard]] static StratifiedSinkageResult evaluateStamp(
        const RutStamp2D& stamp, const StratifiedSoilConfig& config) noexcept;

    /// @brief Gets standard preset for thin mud over solid bedrock.
    /// @return StratifiedSoilConfig configured for mud-on-bedrock.
    [[nodiscard]] static StratifiedSoilConfig getMudOnBedrock() noexcept;

    /// @brief Gets standard preset for loose snow over asphalt pavement.
    /// @return StratifiedSoilConfig configured for snow-on-asphalt.
    [[nodiscard]] static StratifiedSoilConfig getSnowOnAsphalt() noexcept;
};

} // namespace vehicle_sim::terrain
