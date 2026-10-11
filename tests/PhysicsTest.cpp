#include <gtest/gtest.h>

#include <cmath>

#include "libs/VehicleSimCore/physics/ContactSolver2D.h"
#include "libs/VehicleSimCore/physics/Integrator2D.h"
#include "libs/VehicleSimCore/physics/PhysicsWorld2D.h"
#include "libs/VehicleSimCore/physics/RigidBody2D.h"

namespace vehicle_sim::testing {

using vehicle_sim::math::Vec2;
using vehicle_sim::physics::BodyType;
using vehicle_sim::physics::Contact2D;
using vehicle_sim::physics::ContactSolver2D;
using vehicle_sim::physics::Integrator2D;
using vehicle_sim::physics::PhysicsWorld2D;
using vehicle_sim::physics::RigidBody2D;
using vehicle_sim::physics::RigidBodyConfig;

inline constexpr double kPi { 3.14159265358979323846 };
inline constexpr double kEpsilon { 1e-9 };

// -----------------------------------------------------------------------------
// RigidBody2D Tests
// -----------------------------------------------------------------------------

TEST(RigidBody2DTest, DefaultAndConfigConstructors)
{
    const RigidBody2D defaultBody { };
    EXPECT_TRUE(defaultBody.isDynamic());
    EXPECT_DOUBLE_EQ(defaultBody.getMass(), 1.0);
    EXPECT_DOUBLE_EQ(defaultBody.getInverseMass(), 1.0);
    EXPECT_DOUBLE_EQ(defaultBody.getInertia(), 1.0);
    EXPECT_DOUBLE_EQ(defaultBody.getInverseInertia(), 1.0);
    EXPECT_DOUBLE_EQ(defaultBody.getPosition().x, 0.0);
    EXPECT_DOUBLE_EQ(defaultBody.getPosition().y, 0.0);
    EXPECT_DOUBLE_EQ(defaultBody.getOrientation(), 0.0);

    RigidBodyConfig config { };
    config.mass = 500.0;
    config.inertia = 1200.0;
    config.position = Vec2 { 10.0, -5.0 };
    config.orientation = kPi / 4.0;
    config.linearDamping = 0.05;
    config.angularDamping = 0.1;
    config.gravityScale = 0.5;
    config.bodyType = BodyType::Dynamic;

    const RigidBody2D configuredBody { config };
    EXPECT_DOUBLE_EQ(configuredBody.getMass(), 500.0);
    EXPECT_NEAR(configuredBody.getInverseMass(), 1.0 / 500.0, kEpsilon);
    EXPECT_DOUBLE_EQ(configuredBody.getInertia(), 1200.0);
    EXPECT_NEAR(configuredBody.getInverseInertia(), 1.0 / 1200.0, kEpsilon);
    EXPECT_DOUBLE_EQ(configuredBody.getPosition().x, 10.0);
    EXPECT_DOUBLE_EQ(configuredBody.getPosition().y, -5.0);
    EXPECT_DOUBLE_EQ(configuredBody.getOrientation(), kPi / 4.0);
    EXPECT_DOUBLE_EQ(configuredBody.getLinearDamping(), 0.05);
    EXPECT_DOUBLE_EQ(configuredBody.getAngularDamping(), 0.1);
    EXPECT_DOUBLE_EQ(configuredBody.getGravityScale(), 0.5);
}

TEST(RigidBody2DTest, StaticAndKinematicBehavior)
{
    RigidBodyConfig staticConfig { };
    staticConfig.bodyType = BodyType::Static;
    staticConfig.mass = 1000.0;

    RigidBody2D staticBody { staticConfig };
    EXPECT_TRUE(staticBody.isStatic());
    EXPECT_FALSE(staticBody.isDynamic());
    EXPECT_DOUBLE_EQ(staticBody.getInverseMass(), 0.0);
    EXPECT_DOUBLE_EQ(staticBody.getInverseInertia(), 0.0);

    // Forces and impulses must not move a static body
    staticBody.applyForce(Vec2 { 1000.0, 500.0 });
    staticBody.applyLinearImpulse(Vec2 { 200.0, 100.0 });
    EXPECT_DOUBLE_EQ(staticBody.getAccumulatedForce().x, 0.0);
    EXPECT_DOUBLE_EQ(staticBody.getLinearVelocity().x, 0.0);
    EXPECT_DOUBLE_EQ(staticBody.getLinearVelocity().y, 0.0);

    // Switching to static zeroes velocity
    RigidBody2D dynamicBody { };
    dynamicBody.setLinearVelocity(Vec2 { 15.0, 20.0 });
    dynamicBody.setAngularVelocity(3.0);
    dynamicBody.setBodyType(BodyType::Static);
    EXPECT_DOUBLE_EQ(dynamicBody.getLinearVelocity().x, 0.0);
    EXPECT_DOUBLE_EQ(dynamicBody.getLinearVelocity().y, 0.0);
    EXPECT_DOUBLE_EQ(dynamicBody.getAngularVelocity(), 0.0);
}

TEST(RigidBody2DTest, ForceAndTorqueAccumulation)
{
    RigidBody2D body { };
    body.setPosition(Vec2 { 2.0, 3.0 });

    // Apply central force
    body.applyForce(Vec2 { 10.0, 0.0 });
    EXPECT_DOUBLE_EQ(body.getAccumulatedForce().x, 10.0);
    EXPECT_DOUBLE_EQ(body.getAccumulatedForce().y, 0.0);
    EXPECT_DOUBLE_EQ(body.getAccumulatedTorque(), 0.0);

    // Apply off-center force at world point (2, 5) => r = (0, 2)
    // Force = (5, 0) => torque = r x F = (0 * 0) - (2 * 5) = -10 N*m
    body.applyForceAtPoint(Vec2 { 5.0, 0.0 }, Vec2 { 2.0, 5.0 });
    EXPECT_DOUBLE_EQ(body.getAccumulatedForce().x, 15.0);
    EXPECT_DOUBLE_EQ(body.getAccumulatedTorque(), -10.0);

    // Direct torque
    body.applyTorque(4.0);
    EXPECT_DOUBLE_EQ(body.getAccumulatedTorque(), -6.0);

    // Clearing forces
    body.clearForces();
    EXPECT_DOUBLE_EQ(body.getAccumulatedForce().x, 0.0);
    EXPECT_DOUBLE_EQ(body.getAccumulatedForce().y, 0.0);
    EXPECT_DOUBLE_EQ(body.getAccumulatedTorque(), 0.0);
}

TEST(RigidBody2DTest, ApplyForceAtLocalPoint)
{
    RigidBody2D body { };
    body.setPosition(Vec2 { 0.0, 0.0 });
    body.setOrientation(0.0); // Heading along +X

    // Local point at (2, 0) (e.g. front axle)
    // Apply lateral world force in +Y direction of 50 N
    // r = (2, 0), F = (0, 50) => torque = (2 * 50) - (0 * 0) = +100 N*m
    body.applyForceAtLocalPoint(Vec2 { 0.0, 50.0 }, Vec2 { 2.0, 0.0 });
    EXPECT_DOUBLE_EQ(body.getAccumulatedForce().x, 0.0);
    EXPECT_DOUBLE_EQ(body.getAccumulatedForce().y, 50.0);
    EXPECT_DOUBLE_EQ(body.getAccumulatedTorque(), 100.0);
}

TEST(RigidBody2DTest, VelocityAtPointQueries)
{
    RigidBody2D body { };
    body.setPosition(Vec2 { 0.0, 0.0 });
    body.setLinearVelocity(Vec2 { 10.0, 0.0 });
    body.setAngularVelocity(2.0); // 2 rad/s counter-clockwise

    // At center of mass
    const Vec2 vCenter { body.getVelocityAtPoint(Vec2 { 0.0, 0.0 }) };
    EXPECT_DOUBLE_EQ(vCenter.x, 10.0);
    EXPECT_DOUBLE_EQ(vCenter.y, 0.0);

    // At point (0, 3): r = (0, 3)
    // v_pt = v + omega x r = (10, 0) + (-omega * r.y, omega * r.x)
    //      = (10, 0) + (-2 * 3, 2 * 0) = (10 - 6, 0) = (4, 0)
    const Vec2 vTop { body.getVelocityAtPoint(Vec2 { 0.0, 3.0 }) };
    EXPECT_DOUBLE_EQ(vTop.x, 4.0);
    EXPECT_DOUBLE_EQ(vTop.y, 0.0);

    // At local point (0, 3) with orientation 0
    const Vec2 vLocal { body.getVelocityAtLocalPoint(Vec2 { 0.0, 3.0 }) };
    EXPECT_DOUBLE_EQ(vLocal.x, 4.0);
    EXPECT_DOUBLE_EQ(vLocal.y, 0.0);
}

TEST(RigidBody2DTest, ImpulseApplication)
{
    RigidBody2D body { };
    body.setMass(2.0);
    body.setInertia(4.0);
    body.setPosition(Vec2 { 1.0, 1.0 });

    // Linear impulse J = (6, 8)
    // delta_v = J / m = (3, 4)
    body.applyLinearImpulse(Vec2 { 6.0, 8.0 });
    EXPECT_DOUBLE_EQ(body.getLinearVelocity().x, 3.0);
    EXPECT_DOUBLE_EQ(body.getLinearVelocity().y, 4.0);

    // Angular impulse J_theta = 8.0
    // delta_omega = 8.0 / 4.0 = 2.0
    body.applyAngularImpulse(8.0);
    EXPECT_DOUBLE_EQ(body.getAngularVelocity(), 2.0);

    // Impulse at point (1, 3): r = (0, 2)
    // J = (10, 0)
    // delta_v = (10 / 2, 0) = (5, 0)
    // r x J = (0 * 0) - (2 * 10) = -20
    // delta_omega = -20 / 4.0 = -5.0
    body.applyImpulseAtPoint(Vec2 { 10.0, 0.0 }, Vec2 { 1.0, 3.0 });
    EXPECT_DOUBLE_EQ(body.getLinearVelocity().x, 8.0);
    EXPECT_DOUBLE_EQ(body.getAngularVelocity(), -3.0);
}

TEST(RigidBody2DTest, EnergiesAndMomenta)
{
    RigidBody2D body { };
    body.setMass(10.0);
    body.setInertia(5.0);
    body.setLinearVelocity(Vec2 { 3.0, 4.0 }); // |v| = 5, v^2 = 25
    body.setAngularVelocity(2.0); // omega^2 = 4

    // E_trans = 0.5 * 10 * 25 = 125
    EXPECT_DOUBLE_EQ(body.getTranslationalEnergy(), 125.0);
    // E_rot = 0.5 * 5 * 4 = 10
    EXPECT_DOUBLE_EQ(body.getRotationalEnergy(), 10.0);
    // E_kin = 135
    EXPECT_DOUBLE_EQ(body.getKineticEnergy(), 135.0);

    // p = m * v = (30, 40)
    const Vec2 p { body.getLinearMomentum() };
    EXPECT_DOUBLE_EQ(p.x, 30.0);
    EXPECT_DOUBLE_EQ(p.y, 40.0);

    // L = I * omega = 5 * 2 = 10
    EXPECT_DOUBLE_EQ(body.getAngularMomentum(), 10.0);
}

TEST(RigidBody2DTest, CoordinateTransformHelpers)
{
    RigidBody2D body { };
    body.setPosition(Vec2 { 5.0, 10.0 });
    body.setOrientation(kPi / 2.0); // 90 deg CCW

    // Local point (1, 0) in world frame:
    // Rotated by 90 deg = (0, 1), translated by (5, 10) = (5, 11)
    const Vec2 worldPt { body.toWorldPoint(Vec2 { 1.0, 0.0 }) };
    EXPECT_NEAR(worldPt.x, 5.0, kEpsilon);
    EXPECT_NEAR(worldPt.y, 11.0, kEpsilon);

    // Inverse transform
    const Vec2 backLocal { body.toLocalPoint(worldPt) };
    EXPECT_NEAR(backLocal.x, 1.0, kEpsilon);
    EXPECT_NEAR(backLocal.y, 0.0, kEpsilon);

    // Direction vector (ignores translation)
    const Vec2 worldVec { body.toWorldVec(Vec2 { 1.0, 0.0 }) };
    EXPECT_NEAR(worldVec.x, 0.0, kEpsilon);
    EXPECT_NEAR(worldVec.y, 1.0, kEpsilon);

    const Vec2 backVec { body.toLocalVec(worldVec) };
    EXPECT_NEAR(backVec.x, 1.0, kEpsilon);
    EXPECT_NEAR(backVec.y, 0.0, kEpsilon);
}

// -----------------------------------------------------------------------------
// Integrator2D Tests
// -----------------------------------------------------------------------------

TEST(Integrator2DTest, ConstantForceAcceleration)
{
    RigidBody2D body { };
    body.setMass(2.0);
    body.setPosition(Vec2 { 0.0, 0.0 });
    body.setLinearVelocity(Vec2 { 0.0, 0.0 });

    // F = 10 N => a = 5 m/s^2
    body.applyForce(Vec2 { 10.0, 0.0 });
    constexpr double dt { 0.1 };

    Integrator2D::integrate(body, dt);

    // New velocity: v = 0 + 5 * 0.1 = 0.5 m/s
    EXPECT_DOUBLE_EQ(body.getLinearVelocity().x, 0.5);
    EXPECT_DOUBLE_EQ(body.getLinearVelocity().y, 0.0);

    // Symplectic Euler position: x = 0 + v_new * dt = 0 + 0.5 * 0.1 = 0.05 m
    EXPECT_DOUBLE_EQ(body.getPosition().x, 0.05);
    EXPECT_DOUBLE_EQ(body.getPosition().y, 0.0);

    // Accumulators cleared
    EXPECT_DOUBLE_EQ(body.getAccumulatedForce().x, 0.0);
}

TEST(Integrator2DTest, ConstantTorqueAngularAcceleration)
{
    RigidBody2D body { };
    body.setInertia(4.0);
    body.setOrientation(0.0);
    body.setAngularVelocity(0.0);

    // tau = 8 N*m => alpha = 2 rad/s^2
    body.applyTorque(8.0);
    constexpr double dt { 0.5 };

    Integrator2D::integrate(body, dt);

    // omega = 0 + 2 * 0.5 = 1.0 rad/s
    EXPECT_DOUBLE_EQ(body.getAngularVelocity(), 1.0);
    // theta = 0 + 1.0 * 0.5 = 0.5 rad
    EXPECT_DOUBLE_EQ(body.getOrientation(), 0.5);
}

TEST(Integrator2DTest, DampingDecay)
{
    RigidBody2D body { };
    body.setMass(1.0);
    body.setLinearVelocity(Vec2 { 10.0, 0.0 });
    body.setAngularVelocity(10.0);
    body.setLinearDamping(1.0);
    body.setAngularDamping(1.0);

    constexpr double dt { 1.0 };
    Integrator2D::integrate(body, dt);

    // Implicit factor: 1 / (1 + 1 * 1) = 0.5
    EXPECT_DOUBLE_EQ(body.getLinearVelocity().x, 5.0);
    EXPECT_DOUBLE_EQ(body.getAngularVelocity(), 5.0);
}

TEST(Integrator2DTest, AngleNormalization)
{
    EXPECT_NEAR(Integrator2D::normalizeAngle(0.0), 0.0, kEpsilon);
    EXPECT_NEAR(Integrator2D::normalizeAngle(kPi), kPi, kEpsilon);
    EXPECT_NEAR(Integrator2D::normalizeAngle(3.0 * kPi), kPi, kEpsilon);
    EXPECT_NEAR(Integrator2D::normalizeAngle(-3.0 * kPi), -kPi, kEpsilon);
    EXPECT_NEAR(Integrator2D::normalizeAngle(2.0 * kPi), 0.0, kEpsilon);
}

TEST(Integrator2DTest, GravityIntegration)
{
    RigidBody2D body { };
    body.setMass(5.0);
    body.setGravityScale(1.0);
    const Vec2 gravity { 0.0, -9.81 };

    constexpr double dt { 1.0 };
    Integrator2D::integrate(body, dt, gravity);

    EXPECT_DOUBLE_EQ(body.getLinearVelocity().x, 0.0);
    EXPECT_NEAR(body.getLinearVelocity().y, -9.81, kEpsilon);
    EXPECT_NEAR(body.getPosition().y, -9.81, kEpsilon);
}

TEST(Integrator2DTest, ZeroAndNegativeDtSafety)
{
    RigidBody2D body { };
    body.setPosition(Vec2 { 5.0, 5.0 });
    body.setLinearVelocity(Vec2 { 1.0, 1.0 });

    Integrator2D::integrate(body, 0.0);
    EXPECT_DOUBLE_EQ(body.getPosition().x, 5.0);

    Integrator2D::integrate(body, -0.5);
    EXPECT_DOUBLE_EQ(body.getPosition().x, 5.0);
}

TEST(Integrator2DTest, HarmonicOscillatorEnergyConservation)
{
    // A 1D harmonic oscillator: m * x'' = -k * x
    // m = 1, k = 100 => omega_n = 10 rad/s
    RigidBody2D body { };
    body.setMass(1.0);
    body.setPosition(Vec2 { 1.0, 0.0 }); // Initial displacement x = 1
    body.setLinearVelocity(Vec2 { 0.0, 0.0 });
    body.setLinearDamping(0.0);

    constexpr double kSpring { 100.0 };
    constexpr double dt { 0.001 };
    constexpr double initialEnergy { 0.5 * kSpring * 1.0 * 1.0 }; // 50 J

    // Step 1000 steps (1.0 second of simulation = ~1.6 cycles)
    for (int step = 0; step < 1000; ++step) {
        const double x { body.getPosition().x };
        const double springForce { -kSpring * x };
        body.applyForce(Vec2 { springForce, 0.0 });
        Integrator2D::integrate(body, dt);
    }

    const double xFinal { body.getPosition().x };
    const double vFinal { body.getLinearVelocity().x };
    const double finalEnergy { (0.5 * body.getMass() * vFinal * vFinal) + (0.5 * kSpring * xFinal * xFinal) };

    // Symplectic Euler preserves total energy within tight bound (< 0.5% drift)
    // whereas Explicit Euler would have grown exponentially
    EXPECT_NEAR(finalEnergy, initialEnergy, 0.3);
}

// -----------------------------------------------------------------------------
// ContactSolver2D Tests
// -----------------------------------------------------------------------------

TEST(ContactSolver2DTest, ElasticBarrierCollision)
{
    // Body moving in -X towards a vertical wall at left
    RigidBody2D body { };
    body.setMass(2.0);
    body.setPosition(Vec2 { 0.0, 0.0 });
    body.setLinearVelocity(Vec2 { -10.0, 0.0 });

    Contact2D contact { };
    contact.point = Vec2 { 0.0, 0.0 };
    contact.normal = Vec2 { -1.0, 0.0 }; // Normal pointing from body towards wall
    contact.restitution = 1.0; // Perfectly elastic
    contact.friction = 0.0;

    const bool resolved = ContactSolver2D::resolve(body, nullptr, contact);
    EXPECT_TRUE(resolved);

    // Rebound velocity should be exactly +10.0 m/s
    EXPECT_DOUBLE_EQ(body.getLinearVelocity().x, 10.0);
    EXPECT_DOUBLE_EQ(body.getLinearVelocity().y, 0.0);
}

TEST(ContactSolver2DTest, InelasticBarrierCollision)
{
    RigidBody2D body { };
    body.setMass(2.0);
    body.setPosition(Vec2 { 0.0, 0.0 });
    body.setLinearVelocity(Vec2 { -10.0, 0.0 });

    Contact2D contact { };
    contact.point = Vec2 { 0.0, 0.0 };
    contact.normal = Vec2 { -1.0, 0.0 }; // Normal pointing from body towards wall
    contact.restitution = 0.0; // Completely inelastic
    contact.friction = 0.0;

    const bool resolved = ContactSolver2D::resolve(body, nullptr, contact);
    EXPECT_TRUE(resolved);

    EXPECT_NEAR(body.getLinearVelocity().x, 0.0, kEpsilon);
}

TEST(ContactSolver2DTest, RestitutionScaling)
{
    RigidBody2D body { };
    body.setMass(1.0);
    body.setLinearVelocity(Vec2 { 0.0, -20.0 });

    Contact2D contact { };
    contact.point = Vec2 { 0.0, 0.0 };
    contact.normal = Vec2 { 0.0, -1.0 }; // Normal pointing from body towards ground
    contact.restitution = 0.6;
    contact.friction = 0.0;

    const bool resolved = ContactSolver2D::resolve(body, nullptr, contact);
    EXPECT_TRUE(resolved);

    // Rebound: -(-20) * 0.6 = +12 m/s
    EXPECT_NEAR(body.getLinearVelocity().y, 12.0, kEpsilon);
}

TEST(ContactSolver2DTest, CoulombFrictionTangentialResponse)
{
    RigidBody2D body { };
    body.setMass(1.0);
    // Moving diagonally: downward into ground and rightward
    body.setLinearVelocity(Vec2 { 10.0, -10.0 });

    Contact2D contact { };
    contact.point = Vec2 { 0.0, 0.0 };
    contact.normal = Vec2 { 0.0, -1.0 }; // Normal pointing from body towards ground
    contact.restitution = 0.0;
    contact.friction = 0.3; // Friction coeff

    const bool resolved = ContactSolver2D::resolve(body, nullptr, contact);
    EXPECT_TRUE(resolved);

    // Vertical velocity becomes 0
    EXPECT_NEAR(body.getLinearVelocity().y, 0.0, kEpsilon);
    // Tangential speed decelerated by friction
    EXPECT_DOUBLE_EQ(body.getLinearVelocity().x, 7.0);
}

TEST(ContactSolver2DTest, TwoBodyMomentumConservation)
{
    // Body A: mass 2, vel = (10, 0)
    // Body B: mass 3, vel = (0, 0)
    RigidBody2D bodyA { };
    bodyA.setMass(2.0);
    bodyA.setPosition(Vec2 { 0.0, 0.0 });
    bodyA.setLinearVelocity(Vec2 { 10.0, 0.0 });

    RigidBody2D bodyB { };
    bodyB.setMass(3.0);
    bodyB.setPosition(Vec2 { 1.0, 0.0 });
    bodyB.setLinearVelocity(Vec2 { 0.0, 0.0 });

    const Vec2 initialMomentum { bodyA.getLinearMomentum() + bodyB.getLinearMomentum() };
    EXPECT_DOUBLE_EQ(initialMomentum.x, 20.0);

    Contact2D contact { };
    contact.point = Vec2 { 0.5, 0.0 };
    contact.normal = Vec2 { 1.0, 0.0 }; // From A towards B
    contact.restitution = 1.0;
    contact.friction = 0.0;

    const bool resolved = ContactSolver2D::resolve(bodyA, &bodyB, contact);
    EXPECT_TRUE(resolved);

    const Vec2 finalMomentum { bodyA.getLinearMomentum() + bodyB.getLinearMomentum() };
    EXPECT_NEAR(finalMomentum.x, initialMomentum.x, kEpsilon);
    EXPECT_NEAR(finalMomentum.y, initialMomentum.y, kEpsilon);

    // In 1D elastic collision:
    // vA' = (mA - mB)/(mA + mB) * vA = (2 - 3)/5 * 10 = -2 m/s
    // vB' = 2*mA/(mA + mB) * vA = 4/5 * 10 = +8 m/s
    EXPECT_NEAR(bodyA.getLinearVelocity().x, -2.0, kEpsilon);
    EXPECT_NEAR(bodyB.getLinearVelocity().x, 8.0, kEpsilon);
}

TEST(ContactSolver2DTest, OffCenterImpactTorque)
{
    // Body A at (0, 0), stationary
    RigidBody2D body { };
    body.setMass(10.0);
    body.setInertia(20.0);
    body.setPosition(Vec2 { 0.0, 0.0 });
    body.setLinearVelocity(Vec2 { 0.0, 0.0 });
    body.setAngularVelocity(0.0);

    // Impact from projectile moving into body at point (0, 2): r = (0, 2)
    // Moving in -X direction
    RigidBody2D projectile { };
    projectile.setMass(1.0);
    projectile.setPosition(Vec2 { 1.0, 2.0 });
    projectile.setLinearVelocity(Vec2 { -10.0, 0.0 });

    Contact2D contact { };
    contact.point = Vec2 { 0.0, 2.0 };
    contact.normal = Vec2 { -1.0, 0.0 }; // From projectile to body
    contact.restitution = 0.0;
    contact.friction = 0.0;

    const bool resolved = ContactSolver2D::resolve(projectile, &body, contact);
    EXPECT_TRUE(resolved);

    // Body should acquire linear velocity in -X and angular velocity
    EXPECT_LT(body.getLinearVelocity().x, 0.0);
    EXPECT_NE(body.getAngularVelocity(), 0.0);
}

TEST(ContactSolver2DTest, SeparatingBodiesNoImpulse)
{
    RigidBody2D body { };
    body.setLinearVelocity(Vec2 { 10.0, 0.0 }); // Moving away from normal (1, 0)

    Contact2D contact { };
    contact.normal = Vec2 { -1.0, 0.0 }; // Normal points left, body moves right
    contact.restitution = 1.0;

    const bool resolved = ContactSolver2D::resolve(body, nullptr, contact);
    EXPECT_FALSE(resolved);
    EXPECT_DOUBLE_EQ(body.getLinearVelocity().x, 10.0);
}

// -----------------------------------------------------------------------------
// PhysicsWorld2D Tests
// -----------------------------------------------------------------------------

TEST(PhysicsWorld2DTest, AddRemoveClearBodies)
{
    PhysicsWorld2D world { };
    EXPECT_EQ(world.getBodyCount(), 0u);

    RigidBody2D bodyA { };
    RigidBody2D bodyB { };

    world.addBody(bodyA);
    world.addBody(bodyB);
    EXPECT_EQ(world.getBodyCount(), 2u);

    // Duplicate add should be ignored
    world.addBody(bodyA);
    EXPECT_EQ(world.getBodyCount(), 2u);

    world.removeBody(bodyA);
    EXPECT_EQ(world.getBodyCount(), 1u);

    world.clearBodies();
    EXPECT_EQ(world.getBodyCount(), 0u);
}

TEST(PhysicsWorld2DTest, StepWorldGravity)
{
    PhysicsWorld2D world { Vec2 { 0.0, -10.0 } };
    RigidBody2D body { };
    body.setMass(2.0);
    body.setPosition(Vec2 { 0.0, 100.0 });

    world.addBody(body);
    world.step(1.0);

    EXPECT_NEAR(body.getLinearVelocity().y, -10.0, kEpsilon);
    EXPECT_NEAR(body.getPosition().y, 90.0, kEpsilon);
}

TEST(PhysicsWorld2DTest, DeterministicRepeatability)
{
    auto runSimulation = []() {
        PhysicsWorld2D world { Vec2 { 0.0, -9.81 } };
        RigidBody2D bodyA { RigidBodyConfig {
            10.0, 25.0, 0.01, 0.02, 1.0, BodyType::Dynamic, Vec2 { 0.0, 50.0 }, 0.0 } };
        RigidBody2D bodyB { RigidBodyConfig { 5.0, 10.0, 0.0, 0.0, 1.0, BodyType::Dynamic, Vec2 { 10.0, 20.0 }, 0.5 } };
        bodyA.applyForce(Vec2 { 15.0, -5.0 });
        bodyB.applyTorque(12.0);
        world.addBody(bodyA);
        world.addBody(bodyB);

        constexpr double dt { 0.016666666666666666 };
        for (int i = 0; i < 500; ++i) {
            world.step(dt);
        }
        return std::make_pair(bodyA.getPosition(), bodyB.getOrientation());
    };

    const auto run1 = runSimulation();
    const auto run2 = runSimulation();

    // Exact bit-identical determinism
    EXPECT_DOUBLE_EQ(run1.first.x, run2.first.x);
    EXPECT_DOUBLE_EQ(run1.first.y, run2.first.y);
    EXPECT_DOUBLE_EQ(run1.second, run2.second);
}

} // namespace vehicle_sim::testing
