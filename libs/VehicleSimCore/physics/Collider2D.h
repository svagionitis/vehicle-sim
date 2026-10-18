#pragma once

#include <cstdint>
#include <variant>

#include "CompoundShape2D.h"
#include "RigidBody2D.h"
#include "libs/VehicleSimCore/math/AABB2D.h"
#include "libs/VehicleSimCore/math/Capsule2D.h"
#include "libs/VehicleSimCore/math/Circle2D.h"
#include "libs/VehicleSimCore/math/OBB2D.h"
#include "libs/VehicleSimCore/math/Polygon2D.h"

namespace vehicle_sim::physics {

/// @enum ColliderType
/// @brief Identifies the active shape primitive in the collider variant.
enum class ColliderType : std::uint8_t { Circle, OBB, Capsule, Polygon, Compound };

/// @struct ColliderMaterial
/// @brief Physical contact parameters for collision response.
struct ColliderMaterial {
    double restitution { 0.2 }; ///< Elasticity coefficient in range [0.0, 1.0].
    double friction { 0.6 }; ///< Coulomb friction coefficient >= 0.0.
};

/// @class Collider2D
/// @brief Collision geometry volume attached to a RigidBody2D.
/// @details Encapsulates geometric shape primitive, physical contact material,
/// collision filter bits, and sensor trigger mode. Uses std::variant to adhere to MISRA C++:2023.
class Collider2D {
public:
    using ShapeVariant = std::variant<math::Circle2D, math::OBB2D, math::Capsule2D, math::Polygon2D, CompoundShape2D>;

    /// @brief Default constructor creating a default unit circle collider.
    Collider2D() noexcept = default;

    /// @brief Constructs a circular collider.
    explicit Collider2D(const math::Circle2D& circle, RigidBody2D* body = nullptr) noexcept;

    /// @brief Constructs an oriented box collider.
    explicit Collider2D(const math::OBB2D& obb, RigidBody2D* body = nullptr) noexcept;

    /// @brief Constructs a capsule collider.
    explicit Collider2D(const math::Capsule2D& capsule, RigidBody2D* body = nullptr) noexcept;

    /// @brief Constructs a convex polygon collider.
    explicit Collider2D(const math::Polygon2D& polygon, RigidBody2D* body = nullptr) noexcept;

    /// @brief Constructs a multi-primitive compound collider.
    explicit Collider2D(const CompoundShape2D& compound, RigidBody2D* body = nullptr) noexcept;

    /// @brief Gets active collider primitive type.
    [[nodiscard]] ColliderType getType() const noexcept;

    /// @brief Gets reference to internal shape variant.
    [[nodiscard]] const ShapeVariant& getShape() const noexcept;

    /// @brief Sets parent rigid body reference.
    void setBody(RigidBody2D* body) noexcept;

    /// @brief Gets parent rigid body pointer.
    [[nodiscard]] RigidBody2D* getBody() const noexcept;

    /// @brief Sets contact material parameters.
    void setMaterial(const ColliderMaterial& material) noexcept;

    /// @brief Gets contact material parameters.
    [[nodiscard]] const ColliderMaterial& getMaterial() const noexcept;

    /// @brief Sets sensor trigger mode (no collision impulse if true).
    void setSensor(bool isSensor) noexcept;

    /// @brief Queries sensor trigger mode.
    [[nodiscard]] bool isSensor() const noexcept;

    /// @brief Sets collision layer category and mask bits.
    void setCollisionFilter(std::uint16_t category, std::uint16_t mask) noexcept;

    /// @brief Tests if this collider can interact with another collider via filter masks.
    [[nodiscard]] bool canCollideWith(const Collider2D& other) const noexcept;

    /// @brief Computes tight world space AABB for broadphase filtering.
    [[nodiscard]] math::AABB2D computeWorldAABB() const noexcept;

    /// @brief Computes world space OBB representation (if type is OBB).
    [[nodiscard]] math::OBB2D toWorldOBB() const noexcept;

    /// @brief Computes world space Circle representation (if type is Circle).
    [[nodiscard]] math::Circle2D toWorldCircle() const noexcept;

    /// @brief Computes world space Capsule representation (if type is Capsule).
    [[nodiscard]] math::Capsule2D toWorldCapsule() const noexcept;

    /// @brief Computes world space Polygon representation (if type is Polygon).
    [[nodiscard]] math::Polygon2D toWorldPolygon() const noexcept;

    /// @brief Performs a raycast against this collider in world space.
    /// @param[in] ray 2D ray query in world space.
    /// @return RayHit2D if hit, or std::nullopt.
    [[nodiscard]] std::optional<math::RayHit2D> raycast(const math::Ray2D& ray) const noexcept;

    /// @brief Sets the broadphase proxy ID associated with this collider.
    /// @param[in] proxyId Proxy identifier in DynamicBVH2D.
    void setBroadphaseProxyId(std::int32_t proxyId) noexcept;

    /// @brief Gets the broadphase proxy ID associated with this collider.
    /// @return Proxy identifier or -1 if unregistered.
    [[nodiscard]] std::int32_t getBroadphaseProxyId() const noexcept;

    /// @brief Sets whether continuous collision detection is enabled for this collider.
    /// @param[in] enabled True to activate CCD sweeps.
    void setCCDEnabled(bool enabled) noexcept;

    /// @brief Queries whether continuous collision detection is enabled for this collider.
    /// @return True if CCD is enabled.
    [[nodiscard]] bool isCCDEnabled() const noexcept;

private:
    ShapeVariant mShape { math::Circle2D { { 0.0, 0.0 }, 0.5 } };
    RigidBody2D* mBody { nullptr };
    ColliderMaterial mMaterial {};
    std::uint16_t mCategoryBits { 0x0001 };
    std::uint16_t mMaskBits { 0xFFFF };
    std::int32_t mBroadphaseProxyId { -1 };
    bool mIsSensor { false };
    bool mCCDEnabled { false };
};

} // namespace vehicle_sim::physics
