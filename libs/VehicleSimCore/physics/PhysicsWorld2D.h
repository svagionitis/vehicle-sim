#pragma once

#include <cstddef>
#include <vector>

#include "ContactSolver2D.h"
#include "Integrator2D.h"
#include "RigidBody2D.h"
#include "libs/VehicleSimCore/math/Vec2.h"

namespace vehicle_sim::physics {

class FluidField2D;

/// @class PhysicsWorld2D
/// @brief World coordinator managing multiple 2D rigid bodies and global environment properties.
/// @details Stepping the world advances all registered dynamic bodies using Symplectic Euler integration
/// and applies global gravitational forces.
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

    /// @brief Advances all registered rigid bodies by dt seconds.
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
    std::vector<RigidBody2D*> mBodies { };
    const FluidField2D* mFluidField { nullptr };
};

} // namespace vehicle_sim::physics
