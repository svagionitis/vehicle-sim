#include "FluidDrag2D.h"

#include <cmath>

namespace vehicle_sim::physics {

math::Vec2 FluidDrag2D::computeRelativeVel(
    const math::Vec2& bodyVelocity, const math::Vec2& fluidVelocity) noexcept
{
    return bodyVelocity - fluidVelocity;
}

math::Vec2 FluidDrag2D::computeDragForce(
    const math::Vec2& relativeVelocity, double fluidDensity,
    double dragCoeff, double referenceArea) noexcept
{
    const double speed { relativeVelocity.length() };
    if ((speed <= 1e-9) || (fluidDensity <= 0.0) || (dragCoeff <= 0.0) || (referenceArea <= 0.0)) {
        return math::Vec2 { 0.0, 0.0 };
    }

    const double magnitude { 0.5 * fluidDensity * dragCoeff * referenceArea * speed * speed };
    return (relativeVelocity / speed) * (-magnitude);
}

CrosswindForces FluidDrag2D::computeCrosswind(
    const math::Vec2& bodyVelocity, double bodyOrientation,
    const FluidSample2D& fluidSample, const FluidDragProperties& properties) noexcept
{
    const math::Vec2 fwd { std::cos(bodyOrientation), std::sin(bodyOrientation) };
    const math::Vec2 lat { -std::sin(bodyOrientation), std::cos(bodyOrientation) };

    const math::Vec2 vRel { bodyVelocity - fluidSample.velocity };
    const double vRelLong { vRel.dot(fwd) };
    const double vRelLat { vRel.dot(lat) };

    const double frontDragMag {
        0.5 * fluidSample.density * properties.frontalDragCoeff * properties.frontalArea
        * std::abs(vRelLong) * vRelLong
    };
    const double localFx { -frontDragMag };

    const double sideDragMag {
        0.5 * fluidSample.density * properties.sideDragCoeff * properties.sideArea
        * std::abs(vRelLat) * vRelLat
    };
    const double localFy { -sideDragMag };

    const math::Vec2 worldForce { (fwd * localFx) + (lat * localFy) };
    const double yawMoment { localFy * properties.cpOffset };

    CrosswindForces result { };
    result.totalForce = worldForce;
    result.sideForce = localFy;
    result.frontalDrag = localFx;
    result.yawMoment = yawMoment;
    return result;
}

} // namespace vehicle_sim::physics
