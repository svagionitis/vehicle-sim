#include <gtest/gtest.h>

#include "libs/VehicleSimCore/dynamics/ForkliftDynamics.h"
#include "libs/VehicleSimCore/physics/RigidBody2D.h"

namespace vehicle_sim::dynamics {
namespace {

    TEST(ForkliftDynamicsTest, DefaultInitializationAndStability)
    {
        ForkliftDynamics forklift {};
        EXPECT_FALSE(forklift.isAirborne());
        EXPECT_FALSE(forklift.isTippedOver());
        EXPECT_GT(forklift.getRearPivotLoad(), 0.0);
        EXPECT_NEAR(forklift.getRolloverIndex(), 0.0, 1e-4);
        EXPECT_NEAR(forklift.getCompositeCogHeight(), 0.55, 1e-4);
    }

    TEST(ForkliftDynamicsTest, MastHeightIncreasesCogElevation)
    {
        ForkliftDynamics forklift {};
        forklift.setPayloadMass(1000.0); // 1 ton payload
        forklift.setMastHeight(0.0);
        const double baselineCogHeight { forklift.getCompositeCogHeight() };

        forklift.setMastHeight(4.0); // Raise mast 4 meters
        const double elevatedCogHeight { forklift.getCompositeCogHeight() };

        EXPECT_GT(elevatedCogHeight, baselineCogHeight);
    }

    TEST(ForkliftDynamicsTest, ExcessivePayloadTriggersForwardTipOver)
    {
        ForkliftDynamics forklift {};
        physics::RigidBodyConfig bodyConfig {};
        bodyConfig.mass = 3500.0;
        physics::RigidBody2D body { bodyConfig };

        // Set excessive cantilevered payload (e.g. 8000 kg on forks)
        forklift.setPayloadMass(8000.0);
        forklift.step(body, 0.016);

        EXPECT_LE(forklift.getRearPivotLoad(), 0.0);
        EXPECT_TRUE(forklift.isTippedOver());
    }

    TEST(ForkliftDynamicsTest, SpeedGovernorLimitsTraction)
    {
        ForkliftDynamics forklift {};
        physics::RigidBodyConfig bodyConfig {};
        bodyConfig.mass = 3500.0;
        physics::RigidBody2D body { bodyConfig };

        forklift.setThrottle(1.0);

        // At rest: full traction
        body.setLinearVelocity(math::Vec2 { 0.0, 0.0 });
        forklift.step(body, 0.016);
        const double restForce { body.getAccumulatedForce().length() };
        EXPECT_GT(restForce, 5000.0);

        // Near max speed (5.5 m/s): governor cuts drive force
        body.clearForces();
        body.setLinearVelocity(math::Vec2 { 5.4, 0.0 });
        forklift.step(body, 0.016);
        const double governedForce { body.getAccumulatedForce().x };
        EXPECT_LT(governedForce, restForce * 0.2);
    }

    TEST(ForkliftDynamicsTest, LateralRolloverUnderSharpTurnWithHighMast)
    {
        ForkliftDynamics forklift {};
        physics::RigidBodyConfig bodyConfig {};
        bodyConfig.mass = 3500.0;
        physics::RigidBody2D body { bodyConfig };

        forklift.setPayloadMass(1500.0);
        forklift.setMastHeight(4.5); // Max mast height
        body.setLinearVelocity(math::Vec2 { 5.0, 0.0 });
        body.setAngularVelocity(1.5); // Hard pivot turn
        forklift.setSteer(1.0);

        forklift.step(body, 0.016);
        EXPECT_GT(forklift.getRolloverIndex(), 0.5);
    }

} // namespace
} // namespace vehicle_sim::dynamics
