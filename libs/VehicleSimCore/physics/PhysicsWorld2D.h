#pragma once

#include <cstddef>
#include <vector>

#include "BroadphasePair2D.h"
#include "ContactSolver2D.h"
#include "DynamicBVH2D.h"
#include "Integrator2D.h"
#include "Joint2D.h"
#include "RigidBody2D.h"
#include "SimulationSnapshot2D.h"
#include "SpeculativeContact2D.h"
#include "libs/VehicleSimCore/math/Ray2D.h"
#include "libs/VehicleSimCore/math/Vec2.h"

namespace vehicle_sim::terrain {
class Heightfield1D;
}

namespace vehicle_sim::physics {

class Collider2D;
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

    /// @brief Registers a collision volume with the physics world.
    /// @param[in] collider Reference to collider component.
    void addCollider(Collider2D& collider);

    /// @brief Unregisters a collision volume from the physics world.
    /// @param[in] collider Reference to collider component to remove.
    void removeCollider(Collider2D& collider) noexcept;

    /// @brief Clears all registered colliders from the world.
    void clearColliders() noexcept;

    /// @brief Gets the number of registered colliders.
    /// @return Count of active colliders.
    [[nodiscard]] std::size_t getColliderCount() const noexcept;

    /// @brief Registers continuous 1D heightfield terrain with the world.
    /// @param[in] terrain Pointer to terrain instance or nullptr.
    void setTerrain(const terrain::Heightfield1D* terrain) noexcept;

    /// @brief Gets active continuous 1D heightfield terrain pointer.
    /// @return Pointer to Heightfield1D or nullptr.
    [[nodiscard]] const terrain::Heightfield1D* getTerrain() const noexcept;

    /// @brief Performs a fast broadphase-accelerated raycast against all registered colliders.
    /// @param[in] ray 2D world space ray query.
    /// @param[out] outHit RayHit2D containing hit distance, point, and normal.
    /// @param[out] outCollider Reference receiving pointer to struck Collider2D or nullptr.
    /// @return True if a collider was struck within the ray's maximum distance.
    [[nodiscard]] bool raycast(
        const math::Ray2D& ray, math::RayHit2D& outHit, Collider2D*& outCollider) const noexcept;

    /// @brief Gets active broadphase dynamic bounding volume hierarchy tree.
    /// @return Const reference to internal DynamicBVH2D instance.
    [[nodiscard]] const DynamicBVH2D& getBroadphaseTree() const noexcept;

    /// @brief Gets active broadphase dynamic bounding volume hierarchy tree.
    /// @return Mutable reference to internal DynamicBVH2D instance.
    [[nodiscard]] DynamicBVH2D& getBroadphaseTree() noexcept;

    /// @brief Sets whether continuous collision detection sweeps are enabled.
    /// @param[in] enabled True to activate CCD sub-stepping for bullet bodies.
    void setCCDEnabled(bool enabled) noexcept;

    /// @brief Queries whether continuous collision detection is enabled.
    /// @return True if CCD is enabled.
    [[nodiscard]] bool isCCDEnabled() const noexcept;

    /// @brief Sets whether speculative predictive contacts are enabled.
    /// @param[in] enabled True to generate speculative contacts.
    void setSpeculativeEnabled(bool enabled) noexcept;

    /// @brief Queries whether speculative predictive contacts are enabled.
    /// @return True if speculative contacts are enabled.
    [[nodiscard]] bool isSpeculativeEnabled() const noexcept;

    /// @brief Sets maximum sub-step count for continuous collision detection.
    /// @param[in] substeps Maximum sub-steps per simulation step.
    void setMaxCCDSubsteps(std::size_t substeps) noexcept;

    /// @brief Gets maximum sub-step count for continuous collision detection.
    /// @return Maximum sub-step budget.
    [[nodiscard]] std::size_t getMaxCCDSubsteps() const noexcept;

    /// @brief Sets speculative contact detection settings.
    /// @param[in] settings Configuration parameters for speculative contacts.
    void setSpeculativeSettings(const SpeculativeSettings& settings) noexcept;

    /// @brief Gets speculative contact detection settings.
    /// @return Current speculative settings.
    [[nodiscard]] const SpeculativeSettings& getSpeculativeSettings() const noexcept;

    /// @brief Captures a bit-exact snapshot of the current physics simulation state.
    /// @param[in] frameIndex Current simulation frame sequence number.
    /// @param[out] snapshot Destination snapshot structure to populate.
    /// @return True if state was completely captured within capacity limits.
    [[nodiscard]] bool captureSnapshot(std::uint32_t frameIndex,
                                       SimulationSnapshot2D& snapshot) const noexcept;

    /// @brief Restores the physics simulation state from a snapshot.
    /// @param[in] snapshot Valid snapshot to restore into the world.
    /// @return True if restore succeeded, false if checksum mismatch or discrepancy.
    [[nodiscard]] bool restoreSnapshot(const SimulationSnapshot2D& snapshot) noexcept;

    /// @brief Restores world to a past snapshot and deterministically resimulates forward.
    /// @tparam InputFn Callable with signature: void(std::uint32_t frame)
    /// @param[in] snapshot Target historical snapshot.
    /// @param[in] targetFrame Final frame index to reach.
    /// @param[in] timeStep Fixed delta time per resimulation step.
    /// @param[in] applyInputs Callback applying inputs prior to each step.
    /// @return True if rollback and resimulation succeeded.
    template <typename InputFn>
    bool rollbackAndReplay(const SimulationSnapshot2D& snapshot,
                           std::uint32_t targetFrame,
                           double timeStep,
                           InputFn applyInputs) noexcept
    {
        if (!restoreSnapshot(snapshot)) {
            return false;
        }
        for (std::uint32_t f { snapshot.frameIndex }; f < targetFrame; ++f) {
            applyInputs(f);
            step(timeStep);
        }
        return true;
    }

private:
    math::Vec2 mGravity { 0.0, -9.81 };
    std::vector<RigidBody2D*> mBodies {};
    std::vector<Joint2D*> mJoints {};
    std::vector<Collider2D*> mColliders {};
    DynamicBVH2D mBroadphaseTree {};
    mutable std::vector<BroadphasePair2D> mPairBuffer {};
    std::size_t mVelocityIterations { 8 };
    std::size_t mPositionIterations { 3 };
    const FluidField2D* mFluidField { nullptr };
    const terrain::Heightfield1D* mTerrain { nullptr };
    bool mCCDEnabled { true };
    bool mSpeculativeEnabled { true };
    std::size_t mMaxCCDSubsteps { 3 };
    SpeculativeSettings mSpeculativeSettings {};
};

} // namespace vehicle_sim::physics
