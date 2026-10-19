#include "Collider2D.h"
#include <cstdint>

namespace vehicle_sim::physics {

Collider2D::Collider2D(const math::Circle2D& circle, RigidBody2D* body) noexcept
    : mShape { circle }
    , mBody { body }
{
}

Collider2D::Collider2D(const math::OBB2D& obb, RigidBody2D* body) noexcept
    : mShape { obb }
    , mBody { body }
{
}

Collider2D::Collider2D(const math::Capsule2D& capsule, RigidBody2D* body) noexcept
    : mShape { capsule }
    , mBody { body }
{
}

Collider2D::Collider2D(const math::Polygon2D& polygon, RigidBody2D* body) noexcept
    : mShape { polygon }
    , mBody { body }
{
}

Collider2D::Collider2D(const CompoundShape2D& compound, RigidBody2D* body) noexcept
    : mShape { compound }
    , mBody { body }
{
}

ColliderType Collider2D::getType() const noexcept
{
    return static_cast<ColliderType>(mShape.index());
}

const Collider2D::ShapeVariant& Collider2D::getShape() const noexcept
{
    return mShape;
}

void Collider2D::setBody(RigidBody2D* body) noexcept
{
    mBody = body;
}

RigidBody2D* Collider2D::getBody() const noexcept
{
    return mBody;
}

void Collider2D::setMaterial(const ColliderMaterial& material) noexcept
{
    mMaterial = material;
}

const ColliderMaterial& Collider2D::getMaterial() const noexcept
{
    return mMaterial;
}

void Collider2D::setSensor(bool isSensor) noexcept
{
    mIsSensor = isSensor;
}

bool Collider2D::isSensor() const noexcept
{
    return mIsSensor;
}

void Collider2D::setCollisionFilter(std::uint16_t category, std::uint16_t mask) noexcept
{
    mCategoryBits = category;
    mMaskBits = mask;
}

bool Collider2D::canCollideWith(const Collider2D& other) const noexcept
{
    return ((mCategoryBits & other.mMaskBits) != 0U) && ((other.mCategoryBits & mMaskBits) != 0U);
}

math::AABB2D Collider2D::computeWorldAABB() const noexcept
{
    const math::Transform2D tf { (mBody != nullptr) ? mBody->getTransform() : math::Transform2D::identity() };

    return std::visit(
        [&](const auto& s) -> math::AABB2D {
            using T = std::decay_t<decltype(s)>;
            if constexpr (std::is_same_v<T, math::Circle2D>) {
                const math::Vec2 worldCenter { tf.transformPoint(s.center) };
                const math::Vec2 rVec { s.radius, s.radius };
                return math::AABB2D { worldCenter - rVec, worldCenter + rVec };
            } else if constexpr (std::is_same_v<T, math::OBB2D>) {
                const math::Vec2 worldCenter { tf.transformPoint(s.center) };
                const double worldRot { tf.getAngle() + s.rotation };
                const math::OBB2D worldOBB { worldCenter, s.halfExtents, worldRot };
                return worldOBB.computeAABB();
            } else if constexpr (std::is_same_v<T, math::Capsule2D>) {
                const math::Vec2 worldP1 { tf.transformPoint(s.p1) };
                const math::Vec2 worldP2 { tf.transformPoint(s.p2) };
                const math::Capsule2D worldCap { worldP1, worldP2, s.radius };
                return worldCap.computeAABB();
            } else if constexpr (std::is_same_v<T, math::Polygon2D>) {
                const std::size_t count { s.vertexCount() };
                std::array<math::Vec2, math::Polygon2D::kMaxVertices> worldVerts {};
                for (std::size_t i { 0 }; i < count; ++i) {
                    worldVerts[i] = tf.transformPoint(s.getVertex(i));
                }
                const math::Polygon2D worldPoly { worldVerts.data(), count };
                return worldPoly.computeAABB();
            } else if constexpr (std::is_same_v<T, CompoundShape2D>) {
                return s.computeWorldAABB(tf);
            }
        },
        mShape);
}

math::OBB2D Collider2D::toWorldOBB() const noexcept
{
    const math::Transform2D tf { (mBody != nullptr) ? mBody->getTransform() : math::Transform2D::identity() };
    if (const auto* obb = std::get_if<math::OBB2D>(&mShape)) {
        return math::OBB2D { tf.transformPoint(obb->center), obb->halfExtents, tf.getAngle() + obb->rotation };
    }
    return math::OBB2D {};
}

math::Circle2D Collider2D::toWorldCircle() const noexcept
{
    const math::Transform2D tf { (mBody != nullptr) ? mBody->getTransform() : math::Transform2D::identity() };
    if (const auto* circ = std::get_if<math::Circle2D>(&mShape)) {
        return math::Circle2D { tf.transformPoint(circ->center), circ->radius };
    }
    return math::Circle2D {};
}

math::Capsule2D Collider2D::toWorldCapsule() const noexcept
{
    const math::Transform2D tf { (mBody != nullptr) ? mBody->getTransform() : math::Transform2D::identity() };
    if (const auto* cap = std::get_if<math::Capsule2D>(&mShape)) {
        return math::Capsule2D { tf.transformPoint(cap->p1), tf.transformPoint(cap->p2), cap->radius };
    }
    return math::Capsule2D {};
}

math::Polygon2D Collider2D::toWorldPolygon() const noexcept
{
    const math::Transform2D tf { (mBody != nullptr) ? mBody->getTransform() : math::Transform2D::identity() };
    if (const auto* poly = std::get_if<math::Polygon2D>(&mShape)) {
        const std::size_t count { poly->vertexCount() };
        std::array<math::Vec2, math::Polygon2D::kMaxVertices> worldVerts {};
        for (std::size_t i { 0 }; i < count; ++i) {
            worldVerts[i] = tf.transformPoint(poly->getVertex(i));
        }
        return math::Polygon2D { worldVerts.data(), count };
    }
    if (const auto* obb = std::get_if<math::OBB2D>(&mShape)) {
        const math::OBB2D wObb { tf.transformPoint(obb->center), obb->halfExtents, tf.getAngle() + obb->rotation };
        const std::array<math::Vec2, 4> verts { wObb.vertices() };
        return math::Polygon2D { verts.data(), 4 };
    }
    return math::Polygon2D {};
}

std::optional<math::RayHit2D> Collider2D::raycast(const math::Ray2D& ray) const noexcept
{
    const math::Transform2D tf { (mBody != nullptr) ? mBody->getTransform() : math::Transform2D::identity() };

    return std::visit(
        [&](const auto& s) -> std::optional<math::RayHit2D> {
            using T = std::decay_t<decltype(s)>;
            if constexpr (std::is_same_v<T, math::Circle2D>) {
                const math::Circle2D worldCircle { tf.transformPoint(s.center), s.radius };
                return worldCircle.raycast(ray);
            } else if constexpr (std::is_same_v<T, math::OBB2D>) {
                const math::OBB2D worldOBB { tf.transformPoint(s.center), s.halfExtents, tf.getAngle() + s.rotation };
                return worldOBB.raycast(ray);
            } else if constexpr (std::is_same_v<T, math::Capsule2D>) {
                const math::Capsule2D worldCap { tf.transformPoint(s.p1), tf.transformPoint(s.p2), s.radius };
                return worldCap.raycast(ray);
            } else if constexpr (std::is_same_v<T, math::Polygon2D>) {
                const std::size_t count { s.vertexCount() };
                std::array<math::Vec2, math::Polygon2D::kMaxVertices> worldVerts {};
                for (std::size_t i { 0 }; i < count; ++i) {
                    worldVerts[i] = tf.transformPoint(s.getVertex(i));
                }
                const math::Polygon2D worldPoly { worldVerts.data(), count };
                return worldPoly.raycast(ray);
            } else if constexpr (std::is_same_v<T, CompoundShape2D>) {
                std::optional<math::RayHit2D> closestHit { std::nullopt };
                const std::size_t count { s.getChildCount() };
                for (std::size_t i { 0 }; i < count; ++i) {
                    const auto& child { s.getChild(i) };
                    const math::Transform2D childTf { tf.combine(child.localTransform) };
                    const auto childHit { std::visit(
                        [&](const auto& cs) -> std::optional<math::RayHit2D> {
                            using CT = std::decay_t<decltype(cs)>;
                            if constexpr (std::is_same_v<CT, math::Circle2D>) {
                                return math::Circle2D { childTf.transformPoint(cs.center), cs.radius }.raycast(ray);
                            } else if constexpr (std::is_same_v<CT, math::OBB2D>) {
                                return math::OBB2D { childTf.transformPoint(cs.center), cs.halfExtents,
                                    childTf.getAngle() + cs.rotation }
                                    .raycast(ray);
                            } else if constexpr (std::is_same_v<CT, math::Capsule2D>) {
                                return math::Capsule2D { childTf.transformPoint(cs.p1), childTf.transformPoint(cs.p2),
                                    cs.radius }
                                    .raycast(ray);
                            } else if constexpr (std::is_same_v<CT, math::Polygon2D>) {
                                const std::size_t pCount { cs.vertexCount() };
                                std::array<math::Vec2, math::Polygon2D::kMaxVertices> pVerts {};
                                for (std::size_t j { 0 }; j < pCount; ++j) {
                                    pVerts[j] = childTf.transformPoint(cs.getVertex(j));
                                }
                                return math::Polygon2D { pVerts.data(), pCount }.raycast(ray);
                            }
                        },
                        child.shape) };
                    if (childHit.has_value()) {
                        if (!closestHit.has_value() || childHit->distance < closestHit->distance) {
                            closestHit = childHit;
                        }
                    }
                }
                return closestHit;
            }
        },
        mShape);
}

void Collider2D::setBroadphaseProxyId(std::int32_t proxyId) noexcept
{
    mBroadphaseProxyId = proxyId;
}

std::int32_t Collider2D::getBroadphaseProxyId() const noexcept
{
    return mBroadphaseProxyId;
}

void Collider2D::setCCDEnabled(bool enabled) noexcept
{
    mCCDEnabled = enabled;
}

bool Collider2D::isCCDEnabled() const noexcept
{
    return mCCDEnabled;
}

} // namespace vehicle_sim::physics
