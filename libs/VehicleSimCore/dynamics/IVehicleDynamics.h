#pragma once

namespace vehicle_sim::physics {
class RigidBody2D;
} // namespace vehicle_sim::physics

namespace vehicle_sim::dynamics {

/// @class IVehicleDynamics
/// @brief Abstract interface for vehicle dynamics models.
/// @details Defines the contract for dynamics coordinators translating driver/pilot
/// inputs into accumulated forces and torques on a target 2D rigid body.
class IVehicleDynamics {
public:
    /// @brief Virtual destructor ensuring safe polymorphic deletion.
    virtual ~IVehicleDynamics() noexcept = default;

    /// @brief Advances the vehicle dynamics simulation step.
    /// @details Translates internal control inputs into forces applied to the rigid body.
    /// @param[in,out] body Target rigid body to receive accumulated forces and torques.
    /// @param[in] dt Timestep duration in seconds.
    virtual void step(physics::RigidBody2D& body, double dt) noexcept = 0;

    /// @brief Resets transient dynamics states.
    /// @details Clears internal timers, boost levels, or transient slip states without
    /// modifying the attached rigid body's position or momentum.
    virtual void reset() noexcept = 0;

    /// @brief Queries whether the vehicle is airborne.
    /// @return True if the vehicle is detached from the ground surface.
    [[nodiscard]] virtual bool isAirborne() const noexcept = 0;
};

} // namespace vehicle_sim::dynamics
