#pragma once

#include "libs/VehicleSimCore/math/Vec2.h"

namespace vehicle_sim::physics {

class RigidBody2D;

/// @enum JointType
/// @brief Classification of physical 2D multi-body joint constraints.
enum class JointType {
    Distance, ///< Distance constraint maintaining target anchor separation.
    Revolute ///< Pin/hinge joint constraining translational anchors with optional limits and motor.
};

/// @struct JointConfig
/// @brief Base configuration parameters common to all 2D multi-body joints.
struct JointConfig {
    RigidBody2D* bodyA { nullptr }; ///< Pointer to first constrained body (non-owning).
    RigidBody2D* bodyB { nullptr }; ///< Pointer to second constrained body (non-owning).
    math::Vec2 localAnchorA { 0.0, 0.0 }; ///< Anchor offset in body A local coordinates.
    math::Vec2 localAnchorB { 0.0, 0.0 }; ///< Anchor offset in body B local coordinates.
    bool collideConnected { false }; ///< True if linked bodies may collide against each other.
};

/// @class Joint2D
/// @brief Abstract base class for 2D multi-body rigid body constraints.
/// @details Defines the lifecycle and solver interface executed by PhysicsWorld2D.
class Joint2D {
public:
    /// @brief Parameterized constructor initializing base joint linkage.
    /// @param[in] type Concrete joint classification.
    /// @param[in] config Base joint parameters.
    explicit Joint2D(JointType type, const JointConfig& config) noexcept;

    /// @brief Virtual destructor for polymorphic joint destruction.
    virtual ~Joint2D() noexcept = default;

    Joint2D(const Joint2D&) = delete;
    Joint2D& operator=(const Joint2D&) = delete;
    Joint2D(Joint2D&&) noexcept = default;
    Joint2D& operator=(Joint2D&&) noexcept = default;

    /// @brief Gets concrete joint type.
    /// @return JointType enum.
    [[nodiscard]] JointType getType() const noexcept;

    /// @brief Gets pointer to first constrained rigid body.
    /// @return Pointer to body A.
    [[nodiscard]] RigidBody2D* getBodyA() const noexcept;

    /// @brief Gets pointer to second constrained rigid body.
    /// @return Pointer to body B.
    [[nodiscard]] RigidBody2D* getBodyB() const noexcept;

    /// @brief Gets anchor point offset in body A local frame.
    /// @return Anchor vector in local coordinates.
    [[nodiscard]] math::Vec2 getLocalAnchorA() const noexcept;

    /// @brief Gets anchor point offset in body B local frame.
    /// @return Anchor vector in local coordinates.
    [[nodiscard]] math::Vec2 getLocalAnchorB() const noexcept;

    /// @brief Gets anchor point A transformed to world coordinates.
    /// @return World space anchor position.
    [[nodiscard]] math::Vec2 getWorldAnchorA() const noexcept;

    /// @brief Gets anchor point B transformed to world coordinates.
    /// @return World space anchor position.
    [[nodiscard]] math::Vec2 getWorldAnchorB() const noexcept;

    /// @brief Queries whether collision between linked bodies is permitted.
    /// @return True if connected bodies collide.
    [[nodiscard]] bool getCollideConnected() const noexcept;

    /// @brief Pre-calculates effective mass, Baumgarte bias, and applies warm-starting impulses.
    /// @param[in] dt Simulation timestep in seconds.
    virtual void initVelocityConstraints(double dt) noexcept = 0;

    /// @brief Solves velocity-level constraint equations using sequential impulses.
    virtual void solveVelocityConstraints() noexcept = 0;

    /// @brief Solves position-level geometric drift to eliminate anchor separation.
    /// @return True if position constraint error is within tolerance.
    [[nodiscard]] virtual bool solvePositionConstraints() noexcept = 0;

    /// @brief Gets reaction force vector exerted on body B in world coordinates.
    /// @param[in] invDt Inverse timestep (1.0 / dt).
    /// @return Reaction force in Newtons.
    [[nodiscard]] virtual math::Vec2 getReactionForce(double invDt) const noexcept = 0;

    /// @brief Gets reaction torque scalar exerted on body B.
    /// @param[in] invDt Inverse timestep (1.0 / dt).
    /// @return Reaction torque in Newton-meters.
    [[nodiscard]] virtual double getReactionTorque(double invDt) const noexcept = 0;

protected:
    JointType mType { JointType::Distance };
    RigidBody2D* mBodyA { nullptr };
    RigidBody2D* mBodyB { nullptr };
    math::Vec2 mLocalAnchorA { 0.0, 0.0 };
    math::Vec2 mLocalAnchorB { 0.0, 0.0 };
    bool mCollideConnected { false };
};

} // namespace vehicle_sim::physics
