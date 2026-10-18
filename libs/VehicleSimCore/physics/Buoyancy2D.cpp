#include "Buoyancy2D.h"

#include <cmath>

#include "Collider2D.h"

namespace vehicle_sim::physics {

BuoyancyResult2D Buoyancy2D::computeForces(
    const SubmergedSlice2D& slice,
    const math::Vec2& cog,
    const math::Vec2& linearVelocity,
    double angularVelocity,
    double headingAngle,
    const math::Vec2& fluidVelocity,
    const BuoyancyConfig2D& config) noexcept {
    BuoyancyResult2D res {};
    res.state = slice.state;
    res.submergedArea = slice.submergedArea;
    res.centerOfBuoyancy = slice.centerOfBuoyancy;

    if (!slice.isSubmerged() || slice.submergedArea < 1e-8) {
        return res;
    }

    // 1. Archimedes Buoyant Force: F_b = -rho * A_sub * g
    const math::Vec2 buoyantForce = -config.gravity * (config.fluidDensity * slice.submergedArea);
    res.buoyantForce = buoyantForce;

    // 2. Righting Torque about Center of Gravity: tau_b = (C_b - Cog) x F_b
    const math::Vec2 arm = slice.centerOfBuoyancy - cog;
    const double rightingTorque = (arm.x * buoyantForce.y) - (arm.y * buoyantForce.x);
    res.rightingTorque = rightingTorque;

    // 3. Metacentric Height Evaluation (GM = BM + KB - KG)
    if (slice.waterplaneInertia > 0.0 && slice.submergedArea > 1e-6) {
        const double bm = slice.waterplaneInertia / slice.submergedArea;
        const double kb = slice.centerOfBuoyancy.y;
        const double kg = cog.y;
        res.metacentricHeight = bm + kb - kg;
    }

    // 4. Relative Fluid Flow Velocity at Center of Buoyancy
    const math::Vec2 tangentialVel { -angularVelocity * arm.y, angularVelocity * arm.x };
    const math::Vec2 vPoint = linearVelocity + tangentialVel;
    const math::Vec2 vRel = vPoint - fluidVelocity;

    // 5. Anisotropic Form and Skin Drag
    const math::Vec2 forwardHeading { std::cos(headingAngle), std::sin(headingAngle) };
    const math::Vec2 lateralHeading { -std::sin(headingAngle), std::cos(headingAngle) };

    const double vLong = vRel.dot(forwardHeading);
    const double vLat = vRel.dot(lateralHeading);

    const double qLong = 0.5 * config.fluidDensity * config.dragCoefficientLong * slice.submergedArea;
    const double qLat = 0.5 * config.fluidDensity * config.dragCoefficientLat * slice.submergedArea;

    const math::Vec2 dragLong = -forwardHeading * (qLong * vLong * std::abs(vLong));
    const math::Vec2 dragLat = -lateralHeading * (qLat * vLat * std::abs(vLat));
    const math::Vec2 linearSkinDrag = -vRel * (config.linearDamping * slice.submergedArea);

    res.dragForce = dragLong + dragLat + linearSkinDrag;

    // 6. Hydrodynamic Viscous Angular Damping
    const double rotDamp = -angularVelocity * (config.rotationalDamping * config.fluidDensity * slice.submergedArea);
    res.dampingTorque = rotDamp;

    // 7. Accumulate Net Forces and Torques
    res.totalForce = res.buoyantForce + res.dragForce;
    res.totalTorque = res.rightingTorque + res.dampingTorque;

    return res;
}

BuoyancyResult2D Buoyancy2D::applyBuoyancy(
    RigidBody2D& body,
    const math::Polygon2D& polygon,
    const Waterline2D& waterline,
    const BuoyancyConfig2D& config,
    const math::Vec2& fluidVelocity) noexcept {
    const SubmergedSlice2D slice = HullSlicer2D::slicePolygon(polygon, waterline);
    const BuoyancyResult2D result = computeForces(
        slice,
        body.getPosition(),
        body.getLinearVelocity(),
        body.getAngularVelocity(),
        body.getOrientation(),
        fluidVelocity,
        config);

    if (result.state != SubmergenceState::Airborne) {
        body.applyForce(result.totalForce);
        body.applyTorque(result.totalTorque);
    }

    return result;
}

BuoyancyResult2D Buoyancy2D::applyToCollider(
    RigidBody2D& body,
    const Collider2D& collider,
    const Waterline2D& waterline,
    const BuoyancyConfig2D& config,
    const math::Vec2& fluidVelocity) noexcept {
    SubmergedSlice2D slice {};

    std::visit([&slice, &waterline, &body](const auto& shape) noexcept {
        using T = std::decay_t<decltype(shape)>;
        if constexpr (std::is_same_v<T, math::Polygon2D>) {
            slice = HullSlicer2D::slicePolygon(shape, waterline);
        } else if constexpr (std::is_same_v<T, math::OBB2D>) {
            slice = HullSlicer2D::sliceOBB(shape, waterline);
        } else if constexpr (std::is_same_v<T, math::Circle2D>) {
            slice = HullSlicer2D::sliceCircle(shape, waterline);
        } else if constexpr (std::is_same_v<T, math::Capsule2D>) {
            // Approximate capsule by an OBB encompassing both endpoints + radius
            const math::Vec2 diff = shape.p2 - shape.p1;
            const double len = diff.length();
            const math::Vec2 center = (shape.p1 + shape.p2) * 0.5;
            const double rot = std::atan2(diff.y, diff.x);
            const math::OBB2D capsuleBox { center, math::Vec2 { (len * 0.5) + shape.radius, shape.radius }, rot };
            slice = HullSlicer2D::sliceOBB(capsuleBox, waterline);
        } else if constexpr (std::is_same_v<T, CompoundShape2D>) {
            // For compound shape, slice the bounding box approximation
            const math::AABB2D aabb = shape.computeWorldAABB(body.getTransform());
            const math::OBB2D compoundBox { aabb.center(), aabb.halfExtents(), 0.0 };
            slice = HullSlicer2D::sliceOBB(compoundBox, waterline);
        }
    }, collider.getShape());

    const BuoyancyResult2D result = computeForces(
        slice,
        body.getPosition(),
        body.getLinearVelocity(),
        body.getAngularVelocity(),
        body.getOrientation(),
        fluidVelocity,
        config);

    if (result.state != SubmergenceState::Airborne) {
        body.applyForce(result.totalForce);
        body.applyTorque(result.totalTorque);
    }

    return result;
}

} // namespace vehicle_sim::physics
