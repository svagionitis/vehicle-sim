#pragma once

#include <cmath>

#include "libs/VehicleSimCore/math/Mat2.h"
#include "libs/VehicleSimCore/math/Transform2D.h"
#include "libs/VehicleSimCore/math/Vec2.h"

namespace vehicle_sim::physics {

/// @enum BodyType
/// @brief Kinematic classification of a 2D rigid body.
enum class BodyType {
    Dynamic, ///< Fully simulated with mass, forces, and impulses.
    Static, ///< Infinite mass and inertia; immovable by forces.
    Kinematic ///< Controlled directly by velocity; unaffected by forces.
};

/// @struct RigidBodyConfig
/// @brief Initialization parameters for configuring a 2D rigid body.
struct RigidBodyConfig {
    double mass { 1.0 };
    double inertia { 1.0 };
    double linearDamping { 0.0 };
    double angularDamping { 0.0 };
    double gravityScale { 1.0 };
    BodyType bodyType { BodyType::Dynamic };
    math::Vec2 position { 0.0, 0.0 };
    double orientation { 0.0 };
};

/// @class RigidBody2D
/// @brief 2D rigid body representing inertial and kinematic physics state.
/// @details Encapsulates translation, rotation, mass, moment of inertia, and force accumulators
/// without external physics engine dependencies.
class RigidBody2D {
public:
    /// @brief Default constructor creating a unit dynamic body at origin.
    RigidBody2D() noexcept;

    /// @brief Parameterized constructor using configuration settings.
    /// @param[in] config Initial configuration parameters.
    explicit RigidBody2D(const RigidBodyConfig& config) noexcept;

    /// @brief Queries whether the body is dynamic.
    /// @return True if body type is Dynamic.
    [[nodiscard]] bool isDynamic() const noexcept;

    /// @brief Queries whether the body is static.
    /// @return True if body type is Static.
    [[nodiscard]] bool isStatic() const noexcept;

    /// @brief Queries whether the body is kinematic.
    /// @return True if body type is Kinematic.
    [[nodiscard]] bool isKinematic() const noexcept;

    /// @brief Gets the body type.
    /// @return Current BodyType.
    [[nodiscard]] BodyType getBodyType() const noexcept;

    /// @brief Sets the body type.
    /// @param[in] type New BodyType.
    void setBodyType(BodyType type) noexcept;

    /// @brief Gets world position.
    /// @return 2D world position vector.
    [[nodiscard]] math::Vec2 getPosition() const noexcept;

    /// @brief Sets world position.
    /// @param[in] pos New 2D world position vector.
    void setPosition(const math::Vec2& pos) noexcept;

    /// @brief Gets linear velocity in world frame.
    /// @return 2D world velocity vector in m/s.
    [[nodiscard]] math::Vec2 getLinearVelocity() const noexcept;

    /// @brief Sets linear velocity in world frame.
    /// @param[in] vel New 2D world velocity vector in m/s.
    void setLinearVelocity(const math::Vec2& vel) noexcept;

    /// @brief Gets orientation angle in radians.
    /// @return Heading angle in radians.
    [[nodiscard]] double getOrientation() const noexcept;

    /// @brief Sets orientation angle in radians.
    /// @param[in] angleRadians Heading angle in radians.
    void setOrientation(double angleRadians) noexcept;

    /// @brief Gets angular velocity in rad/s.
    /// @return Scalar angular velocity in rad/s.
    [[nodiscard]] double getAngularVelocity() const noexcept;

    /// @brief Sets angular velocity in rad/s.
    /// @param[in] omega Scalar angular velocity in rad/s.
    void setAngularVelocity(double omega) noexcept;

    /// @brief Gets body mass in kg.
    /// @return Body mass in kg.
    [[nodiscard]] double getMass() const noexcept;

    /// @brief Sets body mass in kg.
    /// @details Recalculates inverse mass. Must be > 0 for Dynamic bodies.
    /// @param[in] massKg Mass in kilograms.
    void setMass(double massKg) noexcept;

    /// @brief Gets inverse mass in 1/kg.
    /// @return Inverse mass (0 for static/kinematic bodies).
    [[nodiscard]] double getInverseMass() const noexcept;

    /// @brief Gets moment of inertia in kg*m^2.
    /// @return Scalar rotational inertia.
    [[nodiscard]] double getInertia() const noexcept;

    /// @brief Sets moment of inertia in kg*m^2.
    /// @details Recalculates inverse inertia. Must be > 0 for Dynamic bodies.
    /// @param[in] inertiaVal Rotational inertia in kg*m^2.
    void setInertia(double inertiaVal) noexcept;

    /// @brief Gets inverse moment of inertia in 1/(kg*m^2).
    /// @return Inverse inertia (0 for static/kinematic bodies).
    [[nodiscard]] double getInverseInertia() const noexcept;

    /// @brief Gets linear damping coefficient.
    /// @return Linear damping factor.
    [[nodiscard]] double getLinearDamping() const noexcept;

    /// @brief Sets linear damping coefficient.
    /// @param[in] damping Non-negative damping factor.
    void setLinearDamping(double damping) noexcept;

    /// @brief Gets angular damping coefficient.
    /// @return Angular damping factor.
    [[nodiscard]] double getAngularDamping() const noexcept;

    /// @brief Sets angular damping coefficient.
    /// @param[in] damping Non-negative damping factor.
    void setAngularDamping(double damping) noexcept;

    /// @brief Gets gravity scale factor.
    /// @return Gravity multiplier.
    [[nodiscard]] double getGravityScale() const noexcept;

    /// @brief Sets gravity scale factor.
    /// @param[in] scale Multiplier applied to world gravity.
    void setGravityScale(double scale) noexcept;

    /// @brief Gets the rigid transform (position and orientation).
    /// @return Transform2D in world space.
    [[nodiscard]] math::Transform2D getTransform() const noexcept;

    /// @brief Gets total accumulated force in world frame.
    /// @return Accumulated force vector in Newtons.
    [[nodiscard]] math::Vec2 getAccumulatedForce() const noexcept;

    /// @brief Gets total accumulated torque.
    /// @return Accumulated torque scalar in N*m.
    [[nodiscard]] double getAccumulatedTorque() const noexcept;

    /// @brief Applies a world force through the center of mass.
    /// @param[in] forceWorld Force vector in Newtons (world frame).
    void applyForce(const math::Vec2& forceWorld) noexcept;

    /// @brief Applies a world force at a specific world coordinate point.
    /// @details Computes torque as r x F where r is (pointWorld - position).
    /// @param[in] forceWorld Force vector in Newtons.
    /// @param[in] pointWorld Application point in world coordinates.
    void applyForceAtPoint(const math::Vec2& forceWorld, const math::Vec2& pointWorld) noexcept;

    /// @brief Applies a world force at a specific local coordinate offset.
    /// @details Transforms local point to world offset r = R * pointLocal, then accumulates torque.
    /// @param[in] forceWorld Force vector in Newtons.
    /// @param[in] pointLocal Offset in local chassis coordinates.
    void applyForceAtLocalPoint(const math::Vec2& forceWorld, const math::Vec2& pointLocal) noexcept;

    /// @brief Applies a scalar rotational torque.
    /// @param[in] torqueNm Torque scalar in Newton-meters.
    void applyTorque(double torqueNm) noexcept;

    /// @brief Applies a linear impulse through the center of mass.
    /// @details Instantly modifies linear velocity: v += J / m.
    /// @param[in] impulseWorld Linear impulse vector in N*s.
    void applyLinearImpulse(const math::Vec2& impulseWorld) noexcept;

    /// @brief Applies an angular impulse.
    /// @details Instantly modifies angular velocity: omega += J_theta / I.
    /// @param[in] impulseNmSec Angular impulse scalar in N*m*s.
    void applyAngularImpulse(double impulseNmSec) noexcept;

    /// @brief Applies an impulse at a specific world coordinate point.
    /// @details Updates both linear velocity and angular velocity: omega += (r x J) / I.
    /// @param[in] impulseWorld Linear impulse vector in N*s.
    /// @param[in] pointWorld Application point in world coordinates.
    void applyImpulseAtPoint(const math::Vec2& impulseWorld, const math::Vec2& pointWorld) noexcept;

    /// @brief Clears accumulated forces and torques.
    void clearForces() noexcept;

    /// @brief Computes velocity at a given world coordinate point.
    /// @details v_pt = v + omega x (pointWorld - position).
    /// @param[in] pointWorld Target point in world coordinates.
    /// @return Velocity vector in world coordinates.
    [[nodiscard]] math::Vec2 getVelocityAtPoint(const math::Vec2& pointWorld) const noexcept;

    /// @brief Computes velocity at a given body-local coordinate offset.
    /// @details Transforms local point to world offset r, then returns v + omega x r.
    /// @param[in] pointLocal Target point in local body coordinates.
    /// @return Velocity vector in world coordinates.
    [[nodiscard]] math::Vec2 getVelocityAtLocalPoint(const math::Vec2& pointLocal) const noexcept;

    /// @brief Transforms a local point to world coordinates.
    /// @param[in] pointLocal Point in body frame.
    /// @return Point in world frame.
    [[nodiscard]] math::Vec2 toWorldPoint(const math::Vec2& pointLocal) const noexcept;

    /// @brief Transforms a world point to body-local coordinates.
    /// @param[in] pointWorld Point in world frame.
    /// @return Point in body frame.
    [[nodiscard]] math::Vec2 toLocalPoint(const math::Vec2& pointWorld) const noexcept;

    /// @brief Transforms a local direction vector to world coordinates.
    /// @param[in] vecLocal Direction vector in body frame.
    /// @return Direction vector in world frame.
    [[nodiscard]] math::Vec2 toWorldVec(const math::Vec2& vecLocal) const noexcept;

    /// @brief Transforms a world direction vector to body-local coordinates.
    /// @param[in] vecWorld Direction vector in world frame.
    /// @return Direction vector in body frame.
    [[nodiscard]] math::Vec2 toLocalVec(const math::Vec2& vecWorld) const noexcept;

    /// @brief Computes total translational kinetic energy (0.5 * m * v^2).
    /// @return Translational kinetic energy in Joules.
    [[nodiscard]] double getTranslationalEnergy() const noexcept;

    /// @brief Computes total rotational kinetic energy (0.5 * I * omega^2).
    /// @return Rotational kinetic energy in Joules.
    [[nodiscard]] double getRotationalEnergy() const noexcept;

    /// @brief Computes total kinetic energy (translational + rotational).
    /// @return Total kinetic energy in Joules.
    [[nodiscard]] double getKineticEnergy() const noexcept;

    /// @brief Computes linear momentum (m * v).
    /// @return Linear momentum vector in kg*m/s.
    [[nodiscard]] math::Vec2 getLinearMomentum() const noexcept;

    /// @brief Computes angular momentum (I * omega).
    /// @return Angular momentum scalar in kg*m^2/s.
    [[nodiscard]] double getAngularMomentum() const noexcept;

private:
    void updateInertialCache() noexcept;

    math::Vec2 mPosition { 0.0, 0.0 };
    math::Vec2 mLinearVelocity { 0.0, 0.0 };
    double mOrientation { 0.0 };
    double mAngularVelocity { 0.0 };

    double mMass { 1.0 };
    double mInvMass { 1.0 };
    double mInertia { 1.0 };
    double mInvInertia { 1.0 };

    double mLinearDamping { 0.0 };
    double mAngularDamping { 0.0 };
    double mGravityScale { 1.0 };
    BodyType mBodyType { BodyType::Dynamic };

    math::Vec2 mAccumulatedForce { 0.0, 0.0 };
    double mAccumulatedTorque { 0.0 };
};

} // namespace vehicle_sim::physics
