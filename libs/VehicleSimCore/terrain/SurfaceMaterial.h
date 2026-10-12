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
    Grass ///< Natural turf with moderate grip and higher rolling drag.
};

/// @struct SurfaceMaterial
/// @brief Physical surface traction, drag, and restitution parameters.
/// @details Defines Coulomb friction multiplier, rolling resistance multiplier,
/// normal restitution coefficient, and fluid status.
struct SurfaceMaterial {
    MaterialType type { MaterialType::Tarmac };
    double friction { 1.0 }; ///< Coulomb friction coefficient multiplier.
    double rollingDrag { 1.0 }; ///< Rolling resistance multiplier.
    double restitution { 0.1 }; ///< Collision normal restitution coefficient.
    bool isLiquid { false }; ///< True if medium is penetrable liquid.

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
