#pragma once

#include <cstdint>
#include <string_view>

namespace vehicle_sim::terrain {

/// @enum MaterialType
/// @brief Physical surface classification identifiers.
enum class MaterialType : std::uint8_t {
    Tarmac, ///< Dry asphalt pavement with nominal grip.
    Concrete, ///< Hard concrete pavement with high grip.
    Dirt, ///< Compacted dirt trail with moderate grip and rolling drag.
    Gravel, ///< Loose gravel with reduced lateral traction and high rolling resistance.
    Mud, ///< High drag, low grip viscous mud surface.
    Ice, ///< Low friction ice surface with minimal traction.
    Snow, ///< Packed snow with low friction and moderate drag.
    OilSlick, ///< Hazard surface with near-zero friction.
    Water, ///< Liquid water surface or puddle.
    Grass, ///< Natural turf with moderate grip and higher rolling drag.
    Sand ///< Loose granular sand with reduced lateral traction and high rolling resistance.
};

/// @struct SoilMechanicsConfig
/// @brief Physical terramechanics parameters governing soil deformation and rutting.
/// @details Parameterizes the semi-empirical Bekker pressure-sinkage equation,
/// Hegazy-Bekker bulldozing resistance, and viscoelastic recovery dynamics.
struct SoilMechanicsConfig {
    double cohesionModulus { 1000.0 }; ///< Modulus of soil cohesion kc in N/m^(n+1).
    double frictionModulus { 50000.0 }; ///< Modulus of soil internal friction kphi in N/m^(n+2).
    double sinkageExponent { 0.8 }; ///< Dimensionless sinkage exponent n in range [0.3, 1.5].
    double maxPlasticDepth { 0.25 }; ///< Hardpan refusal depth z_max in meters.
    double plasticRatio { 0.75 }; ///< Ratio of permanent plastic deformation in [0.0, 1.0].
    double recoveryTime { 0.0 }; ///< Viscoelastic relaxation time tau in seconds (0 = permanent).
    double compactionRatio { 0.40 }; ///< Soil volume compaction ratio in [0.0, 1.0].
    double bulkDensity { 1600.0 }; ///< Soil bulk density rho in kg/m^3.
    double cohesion { 2000.0 }; ///< Soil cohesive shear strength c in N/m^2 (Pa).
    double frictionAngle { 0.45 }; ///< Soil internal friction angle phi in radians (~26 deg).
    double hardeningRate { 2.0 }; ///< Multi-pass compaction hardening multiplier.
    bool isDeformable { false }; ///< True if material permits physical sinkage and rutting.
};

/// @struct SurfaceMaterial
/// @brief Physical surface traction, drag, and restitution parameters.
/// @details Defines Coulomb friction multiplier, rolling resistance multiplier,
/// normal restitution coefficient, fluid status, and terramechanics soil properties.
struct SurfaceMaterial {
    MaterialType type { MaterialType::Tarmac };
    double friction { 1.0 }; ///< Coulomb friction coefficient multiplier.
    double rollingDrag { 1.0 }; ///< Rolling resistance multiplier.
    double restitution { 0.1 }; ///< Collision normal restitution coefficient.
    bool isLiquid { false }; ///< True if medium is penetrable liquid.
    SoilMechanicsConfig soil {}; ///< Terramechanics soil properties.

    /// @brief Gets preset physical surface material parameters for a given type.
    /// @param[in] materialType Target material classification.
    /// @return SurfaceMaterial configured with default physical parameters.
    [[nodiscard]] static SurfaceMaterial getPreset(MaterialType materialType) noexcept;

    /// @brief Gets human-readable name of material type.
    /// @param[in] materialType Target material classification.
    /// @return String view containing material name.
    [[nodiscard]] static std::string_view getName(MaterialType materialType) noexcept;
};

} // namespace vehicle_sim::terrain
