#include "Joint2D.h"

#include "RigidBody2D.h"

namespace vehicle_sim::physics {

Joint2D::Joint2D(JointType type, const JointConfig& config) noexcept
    : mType { type }
    , mBodyA { config.bodyA }
    , mBodyB { config.bodyB }
    , mLocalAnchorA { config.localAnchorA }
    , mLocalAnchorB { config.localAnchorB }
    , mCollideConnected { config.collideConnected }
{
}

JointType Joint2D::getType() const noexcept
{
    return mType;
}

RigidBody2D* Joint2D::getBodyA() const noexcept
{
    return mBodyA;
}

RigidBody2D* Joint2D::getBodyB() const noexcept
{
    return mBodyB;
}

math::Vec2 Joint2D::getLocalAnchorA() const noexcept
{
    return mLocalAnchorA;
}

math::Vec2 Joint2D::getLocalAnchorB() const noexcept
{
    return mLocalAnchorB;
}

math::Vec2 Joint2D::getWorldAnchorA() const noexcept
{
    if (mBodyA != nullptr) {
        return mBodyA->toWorldPoint(mLocalAnchorA);
    }
    return mLocalAnchorA;
}

math::Vec2 Joint2D::getWorldAnchorB() const noexcept
{
    if (mBodyB != nullptr) {
        return mBodyB->toWorldPoint(mLocalAnchorB);
    }
    return mLocalAnchorB;
}

bool Joint2D::getCollideConnected() const noexcept
{
    return mCollideConnected;
}

} // namespace vehicle_sim::physics
