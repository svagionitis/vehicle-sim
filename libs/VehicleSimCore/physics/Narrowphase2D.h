#pragma once

#include "Collider2D.h"
#include "CompoundShape2D.h"
#include "ContactManifold2D.h"
#include "libs/VehicleSimCore/math/Capsule2D.h"
#include "libs/VehicleSimCore/math/Circle2D.h"
#include "libs/VehicleSimCore/math/OBB2D.h"
#include "libs/VehicleSimCore/math/Polygon2D.h"
#include "libs/VehicleSimCore/terrain/Heightfield1D.h"

namespace vehicle_sim::physics {

/// @class Narrowphase2D
/// @brief Narrowphase collision detection engine and contact manifold generator.
/// @details Evaluates analytical and SAT collision between primitive shapes, compound shapes,
/// and continuous terrain heightfields. Generates ContactManifold2D instances driving ContactSolver2D.
class Narrowphase2D {
public:
    /// @brief Detects collision between two colliders and generates a contact manifold.
    /// @param[in] colA First collider.
    /// @param[in] colB Second collider.
    /// @param[out] manifold Output manifold receiving up to 2 contact points.
    /// @return True if colliders are touching or penetrating.
    [[nodiscard]] static bool detectColliders(
        const Collider2D& colA, const Collider2D& colB, ContactManifold2D& manifold) noexcept;

    /// @brief Detects collision between two circles.
    [[nodiscard]] static bool detectCircleCircle(const math::Circle2D& a, const math::Circle2D& b,
        const ColliderMaterial& matA, const ColliderMaterial& matB, ContactManifold2D& outManifold) noexcept;

    /// @brief Detects collision between a circle and an oriented box.
    [[nodiscard]] static bool detectCircleOBB(const math::Circle2D& circle, const math::OBB2D& obb,
        const ColliderMaterial& matA, const ColliderMaterial& matB, ContactManifold2D& outManifold) noexcept;

    /// @brief Detects collision between a capsule and a circle.
    [[nodiscard]] static bool detectCapsuleCircle(const math::Capsule2D& cap, const math::Circle2D& circ,
        const ColliderMaterial& matA, const ColliderMaterial& matB, ContactManifold2D& outManifold) noexcept;

    /// @brief Detects collision between two capsules.
    [[nodiscard]] static bool detectCapsuleCapsule(const math::Capsule2D& a, const math::Capsule2D& b,
        const ColliderMaterial& matA, const ColliderMaterial& matB, ContactManifold2D& outManifold) noexcept;

    /// @brief Detects collision between a capsule and an oriented box.
    [[nodiscard]] static bool detectCapsuleOBB(const math::Capsule2D& cap, const math::OBB2D& obb,
        const ColliderMaterial& matA, const ColliderMaterial& matB, ContactManifold2D& outManifold) noexcept;

    /// @brief Detects collision between two oriented boxes via SAT and edge clipping.
    [[nodiscard]] static bool detectOBBOBB(const math::OBB2D& a, const math::OBB2D& b, const ColliderMaterial& matA,
        const ColliderMaterial& matB, ContactManifold2D& outManifold) noexcept;

    /// @brief Detects collision between two convex polygons via SAT and edge clipping.
    [[nodiscard]] static bool detectPolygonPolygon(const math::Polygon2D& a, const math::Polygon2D& b,
        const ColliderMaterial& matA, const ColliderMaterial& matB, ContactManifold2D& outManifold) noexcept;

    /// @brief Detects collision between a compound shape and another collider.
    [[nodiscard]] static bool detectCompound(const CompoundShape2D& comp, const math::Transform2D& compTf,
        const Collider2D& other, ContactManifold2D& outManifold) noexcept;

    /// @brief Detects collision between a collider and continuous 1D heightfield terrain.
    [[nodiscard]] static bool detectTerrain(
        const Collider2D& collider, const terrain::Heightfield1D& terrain, ContactManifold2D& outManifold) noexcept;

    /// @brief Combines physical restitution coefficients of two materials.
    [[nodiscard]] static double combineRestitution(double eA, double eB) noexcept;

    /// @brief Combines Coulomb friction coefficients of two materials.
    [[nodiscard]] static double combineFriction(double muA, double muB) noexcept;
};

} // namespace vehicle_sim::physics
