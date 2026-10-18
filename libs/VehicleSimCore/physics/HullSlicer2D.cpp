#include "HullSlicer2D.h"

#include <cmath>

namespace vehicle_sim::physics {

SubmergedSlice2D HullSlicer2D::slicePolygon(
    const math::Polygon2D& polygon,
    const Waterline2D& waterline) noexcept {
    SubmergedSlice2D result {};
    const std::size_t n = polygon.vertexCount();
    if (n < 3U) {
        return result;
    }

    std::array<math::Vec2, SubmergedSlice2D::kMaxSliceVertices> outVerts {};
    std::size_t outCount { 0U };
    std::array<math::Vec2, 2U> cutPoints {};
    std::size_t cutCount { 0U };

    math::Vec2 vPrev = polygon.getVertex(n - 1U);
    double hPrev = waterline.getSignedDistance(vPrev);

    for (std::size_t i = 0U; i < n; ++i) {
        const math::Vec2 vCurr = polygon.getVertex(i);
        const double hCurr = waterline.getSignedDistance(vCurr);

        const bool prevInside = (hPrev <= 0.0);
        const bool currInside = (hCurr <= 0.0);

        if (prevInside && currInside) {
            // Case 1: In -> In
            if (outCount < SubmergedSlice2D::kMaxSliceVertices) {
                outVerts[outCount++] = vCurr;
            }
        } else if (prevInside && !currInside) {
            // Case 2: In -> Out
            const double diff = hCurr - hPrev;
            const double t = (std::abs(diff) > 1e-12) ? (-hPrev / diff) : 0.0;
            const math::Vec2 intersection = vPrev + (vCurr - vPrev) * t;
            if (outCount < SubmergedSlice2D::kMaxSliceVertices) {
                outVerts[outCount++] = intersection;
            }
            if (cutCount < 2U) {
                cutPoints[cutCount++] = intersection;
            }
        } else if (!prevInside && currInside) {
            // Case 4: Out -> In
            const double diff = hCurr - hPrev;
            const double t = (std::abs(diff) > 1e-12) ? (-hPrev / diff) : 0.0;
            const math::Vec2 intersection = vPrev + (vCurr - vPrev) * t;
            if (outCount < SubmergedSlice2D::kMaxSliceVertices) {
                outVerts[outCount++] = intersection;
            }
            if (cutCount < 2U) {
                cutPoints[cutCount++] = intersection;
            }
            if (outCount < SubmergedSlice2D::kMaxSliceVertices) {
                outVerts[outCount++] = vCurr;
            }
        }
        // Case 3: Out -> Out (emit nothing)

        vPrev = vCurr;
        hPrev = hCurr;
    }

    if (outCount < 3U) {
        result.state = SubmergenceState::Airborne;
        return result;
    }

    for (std::size_t k = 0U; k < outCount; ++k) {
        result.vertices[k] = outVerts[k];
    }
    result.vertexCount = outCount;

    computeMoments(outVerts.data(), outCount, result.submergedArea, result.centerOfBuoyancy);

    if (cutCount >= 2U) {
        result.cutPointA = cutPoints[0U];
        result.cutPointB = cutPoints[1U];
        result.waterplaneWidth = (cutPoints[1U] - cutPoints[0U]).length();
        result.waterplaneInertia = (result.waterplaneWidth * result.waterplaneWidth * result.waterplaneWidth) / 12.0;
        result.state = SubmergenceState::PartiallySubmerged;
    } else {
        result.state = SubmergenceState::FullySubmerged;
    }

    return result;
}

SubmergedSlice2D HullSlicer2D::sliceOBB(
    const math::OBB2D& obb,
    const Waterline2D& waterline) noexcept {
    const math::Polygon2D poly = math::Polygon2D::fromOBB(obb.center, obb.halfExtents, obb.rotation);
    return slicePolygon(poly, waterline);
}

SubmergedSlice2D HullSlicer2D::sliceCircle(
    const math::Circle2D& circle,
    const Waterline2D& waterline,
    std::size_t segments) noexcept {
    const double dist = waterline.getSignedDistance(circle.center);
    if (dist >= circle.radius) {
        SubmergedSlice2D res {};
        res.state = SubmergenceState::Airborne;
        return res;
    }

    constexpr double kPi { 3.14159265358979323846 };
    if (dist <= -circle.radius) {
        SubmergedSlice2D res {};
        res.submergedArea = kPi * circle.radius * circle.radius;
        res.centerOfBuoyancy = circle.center;
        res.state = SubmergenceState::FullySubmerged;
        return res;
    }

    // Partially submerged: polygon approximation
    const std::size_t clampedSegments = (segments < 3U) ? 3U : ((segments > 8U) ? 8U : segments);
    const math::Polygon2D poly = math::Polygon2D::fromEllipse(
        circle.center, circle.radius, circle.radius, 0.0, clampedSegments);
    return slicePolygon(poly, waterline);
}

void HullSlicer2D::computeMoments(
    const math::Vec2* inVertices,
    std::size_t count,
    double& outArea,
    math::Vec2& outCentroid) noexcept {
    outArea = 0.0;
    outCentroid = math::Vec2 { 0.0, 0.0 };

    if (count < 3U || inVertices == nullptr) {
        return;
    }

    double signedAreaAccum { 0.0 };
    double cxAccum { 0.0 };
    double cyAccum { 0.0 };

    for (std::size_t i = 0U; i < count; ++i) {
        const std::size_t next = (i + 1U == count) ? 0U : (i + 1U);
        const math::Vec2 p0 = inVertices[i];
        const math::Vec2 p1 = inVertices[next];

        const double crossProduct = (p0.x * p1.y) - (p1.x * p0.y);
        signedAreaAccum += crossProduct;
        cxAccum += (p0.x + p1.x) * crossProduct;
        cyAccum += (p0.y + p1.y) * crossProduct;
    }

    const double area = 0.5 * std::abs(signedAreaAccum);
    outArea = area;

    if (area > 1e-9 && std::abs(signedAreaAccum) > 1e-12) {
        const double factor = 1.0 / (3.0 * signedAreaAccum);
        outCentroid = math::Vec2 { cxAccum * factor, cyAccum * factor };
    }
}

} // namespace vehicle_sim::physics
