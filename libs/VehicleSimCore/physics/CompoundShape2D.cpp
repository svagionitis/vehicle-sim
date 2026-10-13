#include "CompoundShape2D.h"

namespace vehicle_sim::physics {

bool CompoundShape2D::addChild(const CompoundChild2D::Shape& shape, const math::Transform2D& localTransform) noexcept
{
    if (mCount < kMaxChildren) {
        mChildren[mCount] = CompoundChild2D { shape, localTransform };
        ++mCount;
        return true;
    }
    return false;
}

void CompoundShape2D::clear() noexcept
{
    mCount = 0;
}

std::size_t CompoundShape2D::getChildCount() const noexcept
{
    return mCount;
}

const CompoundChild2D& CompoundShape2D::getChild(std::size_t index) const noexcept
{
    if (mCount == 0) {
        static const CompoundChild2D kDefaultChild {};
        return kDefaultChild;
    }
    return mChildren[index % mCount];
}

math::AABB2D CompoundShape2D::computeWorldAABB(const math::Transform2D& bodyTransform) const noexcept
{
    if (mCount == 0) {
        return math::AABB2D { bodyTransform.position, bodyTransform.position };
    }

    math::AABB2D compositeAABB {};
    bool first { true };

    for (std::size_t i { 0 }; i < mCount; ++i) {
        const CompoundChild2D& child { mChildren[i] };
        const math::Transform2D worldTf { bodyTransform * child.localTransform };

        math::AABB2D childAABB {};
        std::visit(
            [&](const auto& s) {
                using T = std::decay_t<decltype(s)>;
                if constexpr (std::is_same_v<T, math::Circle2D>) {
                    const math::Vec2 worldCenter { worldTf.transformPoint(s.center) };
                    const math::Vec2 rVec { s.radius, s.radius };
                    childAABB = math::AABB2D { worldCenter - rVec, worldCenter + rVec };
                } else if constexpr (std::is_same_v<T, math::OBB2D>) {
                    const math::Vec2 worldCenter { worldTf.transformPoint(s.center) };
                    const double worldRot { worldTf.getAngle() + s.rotation };
                    const math::OBB2D worldOBB { worldCenter, s.halfExtents, worldRot };
                    childAABB = worldOBB.computeAABB();
                } else if constexpr (std::is_same_v<T, math::Capsule2D>) {
                    const math::Vec2 worldP1 { worldTf.transformPoint(s.p1) };
                    const math::Vec2 worldP2 { worldTf.transformPoint(s.p2) };
                    const math::Capsule2D worldCap { worldP1, worldP2, s.radius };
                    childAABB = worldCap.computeAABB();
                } else if constexpr (std::is_same_v<T, math::Polygon2D>) {
                    const std::size_t count { s.vertexCount() };
                    std::array<math::Vec2, math::Polygon2D::kMaxVertices> worldVerts {};
                    for (std::size_t v { 0 }; v < count; ++v) {
                        worldVerts[v] = worldTf.transformPoint(s.getVertex(v));
                    }
                    const math::Polygon2D worldPoly { worldVerts.data(), count };
                    childAABB = worldPoly.computeAABB();
                }
            },
            child.shape);

        if (first) {
            compositeAABB = childAABB;
            first = false;
        } else {
            compositeAABB.encapsulate(childAABB);
        }
    }

    return compositeAABB;
}

} // namespace vehicle_sim::physics
