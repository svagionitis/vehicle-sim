#pragma once

#include <cstddef>
#include <vector>

#include "ContactSolver2D.h"
#include "Integrator2D.h"
#include "Joint2D.h"
#include "RigidBody2D.h"
#include "libs/VehicleSimCore/math/Vec2.h"

namespace vehicle_sim::physics {

class FluidField2D;

/// @class PhysicsWorld2D
/// @brief World coordinator managing multiple 2D rigid bodies, multi-body joints, and environment.
/// @details Advances all registered dynamic bodies using Symplectic Euler integration,
/// applies global gravitational forces, and solves multi-body joint constraints via sequential impulses.
class PhysicsWorld2D {
public:
    /// @brief Default constructor with standard downward gravity (0, -9.81).
    PhysicsWorld2D() noexcept = default;

    /// @brief Parameterized constructor setting custom gravity vector.
    /// @param[in] gravity Global gravity vector in m/s^2.
    explicit PhysicsWorld2D(const math::Vec2& gravity) noexcept;

    /// @brief Gets the global gravity acceleration vector.
    /// @return Gravity vector in m/s^2.
    [[nodiscard]] math::Vec2 getGravity() const noexcept;

    /// @brief Sets the global gravity acceleration vector.
    /// @param[in] gravity New gravity vector in m/s^2.
    void setGravity(const math::Vec2& gravity) noexcept;

    /// @brief Registers a rigid body with the physics world.
    /// @details Non-owning reference. Does not re-add duplicate bodies.
    /// @param[in] body Reference to rigid body to simulate.
    void addBody(RigidBody2D& body);

    /// @brief Unregisters a rigid body from the physics world.
    /// @param[in] body Reference to rigid body to remove.
    void removeBody(RigidBody2D& body) noexcept;

    /// @brief Clears all registered bodies from the world.
    void clearBodies() noexcept;

    /// @brief Gets the count of registered bodies.
    /// @return Number of registered bodies.
    [[nodiscard]] std::size_t getBodyCount() const noexcept;

    /// @brief Registers a joint constraint with the physics world.
    /// @details Non-owning reference. Does not re-add duplicate joints.
    /// @param[in] joint Reference to joint constraint to simulate.
    void addJoint(Joint2D& joint);

    /// @brief Unregisters a joint constraint from the physics world.
    /// @param[in] joint Reference to joint constraint to remove.
    void removeJoint(Joint2D& joint) noexcept;

    /// @brief Clears all registered joints from the world.
    void clearJoints() noexcept;

    /// @brief Gets the count of registered joints.
    /// @return Number of registered joints.
    [[nodiscard]] std::size_t getJointCount() const noexcept;

    /// @brief Gets the number of velocity solver iterations.
    /// @return Number of velocity iterations.
    [[nodiscard]] std::size_t getVelocityIterations() const noexcept;

    /// @brief Sets the number of velocity solver iterations.
    /// @param[in] iterations Number of velocity iterations.
    void setVelocityIterations(std::size_t iterations) noexcept;

    /// @brief Gets the number of position solver iterations.
    /// @return Number of position iterations.
    [[nodiscard]] std::size_t getPositionIterations() const noexcept;

    /// @brief Sets the number of position solver iterations.
    /// @param[in] iterations Number of position iterations.
    void setPositionIterations(std::size_t iterations) noexcept;

    /// @brief Advances all registered rigid bodies and joints by dt seconds.
    /// @param[in] dt Time step in seconds.
    void step(double dt) noexcept;

    /// @brief Advances an individual rigid body using world gravity.
    /// @param[in,out] body Body to integrate.
    /// @param[in] dt Time step in seconds.
    void stepBody(RigidBody2D& body, double dt) const noexcept;

    /// @brief Resolves a collision contact manifold between two bodies.
    /// @param[in,out] bodyA First body.
    /// @param[in,out] bodyB Second body (or nullptr for static boundary).
    /// @param[in] contact Contact geometric data.
    /// @return True if impulse was applied.
    [[nodiscard]] bool resolveCollision(
        RigidBody2D& bodyA, RigidBody2D* bodyB, const Contact2D& contact) const noexcept;

    /// @brief Registers an environmental fluid field with the world.
    /// @details Non-owning reference. Pass nullptr to disable environmental fluid forces.
    /// @param[in] field Pointer to active FluidField2D or nullptr.
    void setFluidField(const FluidField2D* field) noexcept;

    /// @brief Gets active environmental fluid field pointer.
    /// @return Pointer to FluidField2D or nullptr if disabled.
    [[nodiscard]] const FluidField2D* getFluidField() const noexcept;

private:
    math::Vec2 mGravity { 0.0, -9.81 };
    std::vector<RigidBody2D*> mBodies {};
    std::vector<Joint2D*> mJoints {};
    std::size_t mVelocityIterations { 8 };
    std::size_t mPositionIterations { 3 };
    const FluidField2D* mFluidField { nullptr };
};

} // namespace vehicle_sim::physics
