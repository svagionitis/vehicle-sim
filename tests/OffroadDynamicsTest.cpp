#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "libs/VehicleSimCore/dynamics/OffroadBallistics2D.h"
#include "libs/VehicleSimCore/dynamics/OffroadTruckDynamics.h"
#include "libs/VehicleSimCore/physics/Integrator2D.h"
#include "libs/VehicleSimCore/physics/RigidBody2D.h"
#include "libs/VehicleSimCore/terrain/Heightfield1D.h"
#include "libs/VehicleSimCore/terrain/SurfaceMaterial.h"

namespace vehicle_sim::test {

using namespace dynamics;

TEST(OffroadTruckDynamicsTest, InitializationAndDefaults)
{
    OffroadTruckDynamics truck {};
    EXPECT_TRUE(truck.isAirborne());
    EXPECT_FALSE(truck.isGrounded());
    EXPECT_DOUBLE_EQ(truck.getSpeed(), 0.0);

    const auto& config { truck.getConfig() };
    EXPECT_DOUBLE_EQ(config.mass, 1450.0);
    EXPECT_DOUBLE_EQ(config.gravity, 9.81);
    EXPECT_EQ(config.axles.size(), 2U);
    EXPECT_LT(config.axles[0].longitudinalOffset, 0.0); // Rear axle
    EXPECT_GT(config.axles[1].longitudinalOffset, 0.0); // Front axle
}

TEST(OffroadTruckDynamicsTest, SuspensionEquilibriumOnFlatGround)
{
    OffroadTruckDynamics truck {};
    terrain::Heightfield1D terrain {};
    terrain.setBaseHeight(0.0);
    truck.setTerrain(&terrain);

    physics::RigidBodyConfig bodyConfig {};
    bodyConfig.mass = 1450.0;
    bodyConfig.inertia = 1200.0;
    bodyConfig.position = math::Vec2 { 10.0, 0.85 };
    physics::RigidBody2D body { bodyConfig };

    constexpr double dt { 1.0 / 60.0 };
    // Step simulation
    for (int i { 0 }; i < 60; ++i) {
        truck.step(body, dt);
        physics::Integrator2D::integrate(body, dt);
        body.clearForces();
    }

    EXPECT_TRUE(truck.isGrounded());
    EXPECT_FALSE(truck.isAirborne());

    const auto& rearAxle { truck.getAxleState(OffroadTruckDynamics::kRearAxle) };
    const auto& frontAxle { truck.getAxleState(OffroadTruckDynamics::kFrontAxle) };

    EXPECT_TRUE(rearAxle.isGrounded);
    EXPECT_TRUE(frontAxle.isGrounded);
    EXPECT_GT(rearAxle.normalForce, 1000.0);
    EXPECT_GT(frontAxle.normalForce, 1000.0);
    EXPECT_GT(rearAxle.compression, 0.0);
    EXPECT_GT(frontAxle.compression, 0.0);
}

TEST(OffroadTruckDynamicsTest, BumpStopStiffnessActivation)
{
    OffroadTruckDynamics truck {};
    terrain::Heightfield1D terrain {};
    terrain.setBaseHeight(0.0);
    truck.setTerrain(&terrain);

    physics::RigidBodyConfig bodyConfig {};
    bodyConfig.mass = 1450.0;
    bodyConfig.inertia = 1200.0;
    // Force deep compression below bumpStopDistance (0.22m)
    bodyConfig.position = math::Vec2 { 10.0, 0.50 };
    physics::RigidBody2D body { bodyConfig };

    truck.step(body, 1.0 / 60.0);

    const auto& rearAxle { truck.getAxleState(OffroadTruckDynamics::kRearAxle) };
    // Normal force should be substantially larger due to non-linear bump stop
    EXPECT_GT(rearAxle.compression, 0.20);
    EXPECT_GT(rearAxle.normalForce, 15000.0);
}

TEST(OffroadTruckDynamicsTest, FourWheelDriveTractionAndBraking)
{
    OffroadTruckDynamics truck {};
    terrain::Heightfield1D terrain {};
    terrain.setBaseHeight(0.0);
    truck.setTerrain(&terrain);

    physics::RigidBodyConfig bodyConfig {};
    bodyConfig.mass = 1450.0;
    bodyConfig.inertia = 1200.0;
    bodyConfig.position = math::Vec2 { 10.0, 0.85 };
    physics::RigidBody2D body { bodyConfig };

    constexpr double dt { 1.0 / 60.0 };
    // Settle onto suspension
    for (int i { 0 }; i < 30; ++i) {
        truck.step(body, dt);
        physics::Integrator2D::integrate(body, dt);
        body.clearForces();
    }

    // Apply positive throttle
    OffroadTruckInput input {};
    input.throttle = 1.0;
    truck.setInput(input);

    for (int i { 0 }; i < 60; ++i) {
        truck.step(body, dt);
        physics::Integrator2D::integrate(body, dt);
        body.clearForces();
    }

    EXPECT_GT(truck.getSpeed(), 2.0);
    EXPECT_GT(body.getLinearVelocity().x, 2.0);

    // Apply braking
    input.throttle = 0.0;
    input.brake = 1.0;
    truck.setInput(input);

    const double initialSpeed { truck.getSpeed() };
    for (int i { 0 }; i < 30; ++i) {
        truck.step(body, dt);
        physics::Integrator2D::integrate(body, dt);
        body.clearForces();
    }

    EXPECT_LT(truck.getSpeed(), initialSpeed);
}

TEST(OffroadTruckDynamicsTest, JumpImpulseAndAirborneAttitudeStabilization)
{
    OffroadTruckDynamics truck {};
    terrain::Heightfield1D terrain {};
    terrain.setBaseHeight(0.0);
    truck.setTerrain(&terrain);

    physics::RigidBodyConfig bodyConfig {};
    bodyConfig.mass = 1450.0;
    bodyConfig.inertia = 1200.0;
    bodyConfig.position = math::Vec2 { 10.0, 0.85 };
    physics::RigidBody2D body { bodyConfig };

    constexpr double dt { 1.0 / 60.0 };
    // Settle
    for (int i { 0 }; i < 30; ++i) {
        truck.step(body, dt);
        physics::Integrator2D::integrate(body, dt);
        body.clearForces();
    }

    EXPECT_TRUE(truck.isGrounded());

    // Trigger jump
    truck.triggerJump();
    truck.step(body, dt);
    physics::Integrator2D::integrate(body, dt);
    body.clearForces();

    EXPECT_GT(body.getLinearVelocity().y, 5.0);
    EXPECT_TRUE(truck.isAirborne());

    // Disturb orientation while airborne
    body.setOrientation(0.35); // ~20 deg pitch up
    body.setAngularVelocity(0.10);

    // Step in-flight attitude stabilizer
    truck.step(body, dt);
    physics::Integrator2D::integrate(body, dt);
    body.clearForces();

    // Corrective torque should pull orientation back down towards zero
    EXPECT_LT(body.getAngularVelocity(), 0.10);
}

TEST(OffroadTruckDynamicsTest, JumpImpulseDoesNotStackWhenHeld)
{
    OffroadTruckDynamics truck {};
    terrain::Heightfield1D terrain {};
    terrain.setBaseHeight(0.0);
    truck.setTerrain(&terrain);

    physics::RigidBodyConfig bodyConfig {};
    bodyConfig.mass = 1450.0;
    bodyConfig.inertia = 1200.0;
    bodyConfig.position = math::Vec2 { 10.0, 0.85 };
    physics::RigidBody2D body { bodyConfig };

    constexpr double dt { 1.0 / 60.0 };
    for (int i { 0 }; i < 30; ++i) {
        truck.step(body, dt);
        physics::Integrator2D::integrate(body, dt);
        body.clearForces();
    }

    OffroadTruckInput input {};
    input.jump = true; // Hold jump button down
    truck.setInput(input);

    truck.step(body, dt);
    physics::Integrator2D::integrate(body, dt);
    body.clearForces();

    const double initialVy { body.getLinearVelocity().y };

    // Step 2 more frames with jump still held
    for (int i { 0 }; i < 2; ++i) {
        truck.step(body, dt);
        physics::Integrator2D::integrate(body, dt);
        body.clearForces();
    }

    // Vertical velocity must decrease under gravity, not double or stack!
    EXPECT_LT(body.getLinearVelocity().y, initialVy);
    EXPECT_LT(body.getLinearVelocity().y, 10.0);
}

TEST(OffroadTruckDynamicsTest, AxleKinematicsUnderChassisPitch)
{
    OffroadTruckDynamics truck {};
    physics::RigidBodyConfig bodyConfig {};
    bodyConfig.mass = 1450.0;
    bodyConfig.inertia = 1200.0;
    bodyConfig.position = math::Vec2 { 10.0, 5.0 };
    bodyConfig.orientation = 0.20; // ~11.5 deg pitch
    physics::RigidBody2D body { bodyConfig };

    truck.step(body, 1.0 / 60.0);

    const auto& frontAxle { truck.getAxleState(OffroadTruckDynamics::kFrontAxle) };
    const math::Vec2 expectedAttach { body.toWorldPoint(math::Vec2 { 1.20, 0.0 }) };
    const double cosT { std::cos(0.20) };
    const double sinT { std::sin(0.20) };
    const math::Vec2 downwardDir { sinT, -cosT };
    const math::Vec2 expectedHub { expectedAttach + (downwardDir * truck.getConfig().axles[1].restLength) };

    EXPECT_NEAR(frontAxle.worldPos.x, expectedHub.x, 1e-3);
    EXPECT_NEAR(frontAxle.worldPos.y, expectedHub.y, 1e-3);
}

TEST(OffroadTruckDynamicsTest, TerramechanicBekkerSinkageDrag)
{
    OffroadTruckDynamics truck {};
    terrain::Heightfield1D terrain {};
    terrain.setBaseHeight(0.0);

    // Configure soft sand soil material
    terrain::SurfaceMaterial sandMat {};
    sandMat.soil.isDeformable = true;
    sandMat.soil.frictionModulus = 80000.0;
    sandMat.soil.sinkageExponent = 0.90;
    sandMat.soil.cohesionModulus = 1500.0;
    sandMat.soil.maxPlasticDepth = 0.20;
    sandMat.friction = 0.80;

    terrain.setDefaultMaterial(sandMat);
    truck.setTerrain(&terrain);

    physics::RigidBodyConfig bodyConfig {};
    bodyConfig.mass = 1450.0;
    bodyConfig.inertia = 1200.0;
    bodyConfig.position = math::Vec2 { 10.0, 0.85 };
    physics::RigidBody2D body { bodyConfig };

    truck.step(body, 1.0 / 60.0);

    const auto& rearAxle { truck.getAxleState(OffroadTruckDynamics::kRearAxle) };
    EXPECT_TRUE(rearAxle.isGrounded);
    EXPECT_GT(rearAxle.soilSinkage, 0.0);
    EXPECT_GT(rearAxle.soilDragForce, 0.0);
}

TEST(OffroadBallistics2DTest, RocketTrajectoryAndPropagation)
{
    OffroadBallistics2D ballistics {};
    EXPECT_EQ(ballistics.getActiveCount(), 0U);

    const math::Vec2 origin { 10.0, 2.0 };
    const double heading { 0.0 }; // Horizontal forward
    const math::Vec2 baseVel { 5.0, 0.0 };

    ballistics.fireRocket(origin, heading, baseVel);
    EXPECT_EQ(ballistics.getActiveCount(), 1U);

    const auto& projs { ballistics.getProjectiles() };
    EXPECT_EQ(projs[0].type, ProjectileType::ForwardRocket);
    EXPECT_DOUBLE_EQ(projs[0].position.x, 10.0);
    EXPECT_GT(projs[0].velocity.x, 40.0); // Muzzle velocity + base velocity

    // Step flight
    ballistics.step(0.10, nullptr);
    EXPECT_GT(projs[0].position.x, 14.0);
    EXPECT_GT(projs[0].age, 0.09);
    EXPECT_TRUE(projs[0].isAlive);
}

TEST(OffroadBallistics2DTest, MortarParabolicArcUnderGravity)
{
    OffroadBallistics2D ballistics {};
    const math::Vec2 origin { 5.0, 1.5 };
    const math::Vec2 baseVel { 0.0, 0.0 };

    ballistics.fireMortar(origin, baseVel);
    EXPECT_EQ(ballistics.getActiveCount(), 1U);

    const auto& projs { ballistics.getProjectiles() };
    EXPECT_EQ(projs[0].type, ProjectileType::AntiAirMortar);
    EXPECT_GT(projs[0].velocity.y, 15.0); // High vertical launch component

    // Advance 0.5s into flight
    for (int i { 0 }; i < 30; ++i) {
        ballistics.step(1.0 / 60.0, nullptr);
    }

    EXPECT_GT(projs[0].position.y, 4.0);
    EXPECT_LT(projs[0].velocity.y, 25.0); // Gravity should decelerate vertical climb
}

TEST(OffroadBallistics2DTest, GroundImpactDetonationAndBlastEvents)
{
    OffroadBallistics2D ballistics {};
    terrain::Heightfield1D terrain {};
    terrain.setBaseHeight(0.0); // Ground at y=0.0

    // Fire rocket slightly angled downward toward ground
    ballistics.fireRocket(math::Vec2 { 10.0, 0.5 }, -0.20, math::Vec2 { 0.0, 0.0 });

    // Step until impact
    for (int i { 0 }; i < 60; ++i) {
        ballistics.step(1.0 / 60.0, &terrain);
    }

    // Rocket should have hit terrain and been removed from active list
    EXPECT_EQ(ballistics.getActiveCount(), 0U);

    // Consume blast events
    auto blasts { ballistics.consumeBlasts() };
    ASSERT_EQ(blasts.size(), 1U);
    EXPECT_NEAR(blasts[0].epicenter.y, 0.0, 0.05);
    EXPECT_GT(blasts[0].radius, 1.0);
    EXPECT_GT(blasts[0].maxImpulse, 1000.0);
}

TEST(OffroadBallistics2DTest, BlastImpulseApplicationToRigidBodies)
{
    OffroadBallistics2D ballistics {};
    physics::RigidBodyConfig bodyConfig {};
    bodyConfig.mass = 100.0;
    bodyConfig.inertia = 50.0;
    bodyConfig.position = math::Vec2 { 5.0, 1.0 };
    physics::RigidBody2D body { bodyConfig };

    BlastEvent blast {};
    blast.epicenter = math::Vec2 { 4.0, 1.0 }; // 1.0m to the left of body
    blast.radius = 3.0;
    blast.maxImpulse = 3000.0;

    std::vector<physics::RigidBody2D*> bodies { &body };
    ballistics.applyBlast(blast, bodies);

    // Body should receive impulse to the right (+X)
    EXPECT_GT(body.getLinearVelocity().x, 5.0);
    EXPECT_NEAR(body.getLinearVelocity().y, 0.0, 0.1);
}

TEST(OffroadBallistics2DTest, ManualDetonationAndReset)
{
    OffroadBallistics2D ballistics {};
    ballistics.fireRocket(math::Vec2 { 0.0, 10.0 }, 0.0, math::Vec2 { 0.0, 0.0 });
    ballistics.fireMortar(math::Vec2 { 0.0, 10.0 }, math::Vec2 { 0.0, 0.0 });

    EXPECT_EQ(ballistics.getActiveCount(), 2U);

    // Manually detonate first projectile
    ballistics.detonate(0);
    ballistics.step(0.01, nullptr);

    EXPECT_EQ(ballistics.getActiveCount(), 1U);
    auto blasts { ballistics.consumeBlasts() };
    EXPECT_EQ(blasts.size(), 1U);

    // Full reset
    ballistics.reset();
    EXPECT_EQ(ballistics.getActiveCount(), 0U);
}

} // namespace vehicle_sim::test
