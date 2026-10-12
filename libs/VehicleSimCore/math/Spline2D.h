#pragma once

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

#include "libs/VehicleSimCore/math/Vec2.h"

namespace vehicle_sim::math {

/// @enum SplineKnotMode
/// @brief Parameterization method for knot intervals in Catmull-Rom splines.
enum class SplineKnotMode : std::uint8_t {
    Uniform, ///< Uniform spacing (alpha = 0.0); standard cardinal spline.
    Centripetal, ///< Centripetal spacing (alpha = 0.5); eliminates cusps and self-intersections.
    Chordal ///< Chordal spacing (alpha = 1.0); proportional to Euclidean distance.
};

/// @struct SplineSample2D
/// @brief Evaluated geometric state at a specific spline parameter.
struct SplineSample2D {
    Vec2 point { 0.0, 0.0 }; ///< World position P(t) in meters.
    Vec2 tangent { 1.0, 0.0 }; ///< Unit forward tangent vector T(t).
    Vec2 normal { 0.0, 1.0 }; ///< Unit upward/leftward normal vector N(t) = (-Ty, Tx).
    Vec2 velocity { 0.0, 0.0 }; ///< Analytical first derivative dP/dt.
    Vec2 acceleration { 0.0, 0.0 }; ///< Analytical second derivative d2P/dt2.
    double curvature { 0.0 }; ///< Signed curvature kappa(t) in 1/m.
    double distance { 0.0 }; ///< Arc-length distance from spline start in meters.
};

/// @struct SplineClosestPoint
/// @brief Result of projecting a world query point onto the spline curve.
struct SplineClosestPoint {
    Vec2 point { 0.0, 0.0 }; ///< Closest point coordinates on spline in meters.
    double distance { 0.0 }; ///< Orthogonal Euclidean distance to query point in meters.
    double arcLength { 0.0 }; ///< Arc-length from spline start to projected point in meters.
    double parameter { 0.0 }; ///< Global spline evaluation parameter t.
    double signedOffset { 0.0 }; ///< Signed lateral offset (positive = left, negative = right).
};

/// @class Spline2D
/// @brief 2D parametric Catmull-Rom spline with analytical derivatives and arc-length parameterization.
/// @details Supports centripetal, uniform, and chordal knot parameterizations for open and closed loops.
/// Evaluates analytical first and second derivatives, signed curvature, Frenet-Serret frames,
/// Gauss-Legendre quadrature arc-length parameterization, and closest-point projections.
class Spline2D {
public:
    /// @brief Default constructor creating an empty spline.
    Spline2D() noexcept = default;

    /// @brief Constructs and initializes spline with control nodes.
    /// @param[in] controlPoints List of control vertices in order.
    /// @param[in] isClosed True if spline forms a closed continuous loop.
    /// @param[in] mode Knot parameterization mode (defaults to Centripetal).
    Spline2D(const std::vector<Vec2>& controlPoints, bool isClosed = false,
        SplineKnotMode mode = SplineKnotMode::Centripetal);

    /// @brief Configures spline control points and updates arc-length lookup tables.
    /// @param[in] controlPoints List of control vertices in order.
    /// @param[in] isClosed True if spline forms a closed continuous loop.
    /// @param[in] mode Knot parameterization mode.
    void setControlPoints(const std::vector<Vec2>& controlPoints, bool isClosed = false,
        SplineKnotMode mode = SplineKnotMode::Centripetal);

    /// @brief Clears all control points and resets spline state.
    void clear() noexcept;

    /// @brief Gets total number of input control points.
    /// @return Control point count.
    [[nodiscard]] std::size_t getPointCount() const noexcept;

    /// @brief Gets total number of evaluable spline segments.
    /// @return Number of polynomial segments.
    [[nodiscard]] std::size_t getSegmentCount() const noexcept;

    /// @brief Queries whether spline forms a closed loop.
    /// @return True if closed loop.
    [[nodiscard]] bool isClosed() const noexcept;

    /// @brief Gets active knot parameterization mode.
    /// @return Knot mode.
    [[nodiscard]] SplineKnotMode getKnotMode() const noexcept;

    /// @brief Gets total integrated arc-length of the spline.
    /// @return Total length in meters.
    [[nodiscard]] double getTotalLength() const noexcept;

    /// @brief Evaluates world position at global parameter t.
    /// @param[in] t Global evaluation parameter in [0, getSegmentCount()].
    /// @return Interpolated 2D Cartesian position.
    [[nodiscard]] Vec2 evaluate(double t) const noexcept;

    /// @brief Evaluates complete geometric state sample at parameter t.
    /// @param[in] t Global evaluation parameter in [0, getSegmentCount()].
    /// @return Full geometric sample including tangents, normal, derivatives, and curvature.
    [[nodiscard]] SplineSample2D evaluateSample(double t) const noexcept;

    /// @brief Evaluates complete geometric state sample at arc-length distance s.
    /// @param[in] s Arc-length from start in meters.
    /// @return Full geometric sample evaluated at distance s.
    [[nodiscard]] SplineSample2D evaluateAtDistance(double s) const noexcept;

    /// @brief Computes signed curvature at parameter t.
    /// @param[in] t Global evaluation parameter.
    /// @return Signed curvature kappa in 1/m (positive = curving left, negative = curving right).
    [[nodiscard]] double getCurvature(double t) const noexcept;

    /// @brief Projects a 2D world query point onto the spline curve.
    /// @param[in] query Point in world Cartesian coordinates.
    /// @return Projection details including closest point, distance, and signed lateral offset.
    [[nodiscard]] SplineClosestPoint findClosest(const Vec2& query) const noexcept;

    /// @brief Discretizes spline into uniformly spaced polyline vertices.
    /// @param[in] stepSize Target spacing between adjacent vertices in meters.
    /// @return Vector of discretized 2D world points.
    [[nodiscard]] std::vector<Vec2> discretize(double stepSize) const;

private:
    struct SegmentKnotData {
        Vec2 p0 { 0.0, 0.0 };
        Vec2 p1 { 0.0, 0.0 };
        Vec2 p2 { 0.0, 0.0 };
        Vec2 p3 { 0.0, 0.0 };
        double t0 { 0.0 };
        double t1 { 1.0 };
        double t2 { 2.0 };
        double t3 { 3.0 };
        double arcLength { 0.0 };
    };

    void buildSegments();
    void buildArcLengthTable();

    [[nodiscard]] double computeAlpha() const noexcept;
    [[nodiscard]] double mapDistanceToParameter(double s) const noexcept;
    [[nodiscard]] double computeSegmentArcLength(std::size_t segIndex, double uEnd) const noexcept;

    std::vector<Vec2> mControlPoints {};
    std::vector<SegmentKnotData> mSegments {};
    std::vector<double> mCumulativeLengths {};
    double mTotalLength { 0.0 };
    bool mIsClosed { false };
    SplineKnotMode mKnotMode { SplineKnotMode::Centripetal };
};

} // namespace vehicle_sim::math
