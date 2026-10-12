#include "Spline2D.h"

#include <algorithm>
#include <array>
#include <cmath>

namespace vehicle_sim::math {

namespace {

    constexpr double kMinKnotDelta { 1e-6 };
    constexpr double kEpsilon { 1e-12 };

    constexpr std::array<double, 5> kGaussNodes { 0.0, -0.53846931010568309, 0.53846931010568309, -0.90617984593866403,
        0.90617984593866403 };

    constexpr std::array<double, 5> kGaussWeights { 0.56888888888888889, 0.47862867049936647, 0.47862867049936647,
        0.23692688505618909, 0.23692688505618909 };

    struct BarryGoldmanEval {
        Vec2 position { 0.0, 0.0 };
        Vec2 velocity { 0.0, 0.0 };
        Vec2 acceleration { 0.0, 0.0 };
    };

    BarryGoldmanEval evaluatePolynomial(const Vec2& p0, const Vec2& p1, const Vec2& p2, const Vec2& p3, double t0,
        double t1, double t2, double t3, double u) noexcept
    {
        const double clampedU { std::clamp(u, 0.0, 1.0) };
        const double dtSegment { t2 - t1 };
        const double t { t1 + (clampedU * dtSegment) };

        const double invT10 { 1.0 / (t1 - t0) };
        const double invT21 { 1.0 / (t2 - t1) };
        const double invT32 { 1.0 / (t3 - t2) };
        const double invT20 { 1.0 / (t2 - t0) };
        const double invT31 { 1.0 / (t3 - t1) };

        const Vec2 a1 { ((t1 - t) * p0 + (t - t0) * p1) * invT10 };
        const Vec2 a2 { ((t2 - t) * p1 + (t - t1) * p2) * invT21 };
        const Vec2 a3 { ((t3 - t) * p2 + (t - t2) * p3) * invT32 };

        const Vec2 b1 { ((t2 - t) * a1 + (t - t0) * a2) * invT20 };
        const Vec2 b2 { ((t3 - t) * a2 + (t - t1) * a3) * invT31 };

        const Vec2 c { ((t2 - t) * b1 + (t - t1) * b2) * invT21 };

        const Vec2 da1 { (p1 - p0) * invT10 };
        const Vec2 da2 { (p2 - p1) * invT21 };
        const Vec2 da3 { (p3 - p2) * invT32 };

        const Vec2 db1 { (a2 - a1 + (t2 - t) * da1 + (t - t0) * da2) * invT20 };
        const Vec2 db2 { (a3 - a2 + (t3 - t) * da2 + (t - t1) * da3) * invT31 };

        const Vec2 dc { (b2 - b1 + (t2 - t) * db1 + (t - t1) * db2) * invT21 };

        const Vec2 d2b1 { (2.0 * (da2 - da1)) * invT20 };
        const Vec2 d2b2 { (2.0 * (da3 - da2)) * invT31 };

        const Vec2 d2c { (2.0 * (db2 - db1) + (t2 - t) * d2b1 + (t - t1) * d2b2) * invT21 };

        BarryGoldmanEval result {};
        result.position = c;
        result.velocity = dc * dtSegment;
        result.acceleration = d2c * (dtSegment * dtSegment);
        return result;
    }

} // namespace

Spline2D::Spline2D(const std::vector<Vec2>& controlPoints, bool isClosed, SplineKnotMode mode)
    : mControlPoints { controlPoints }
    , mIsClosed { isClosed }
    , mKnotMode { mode }
{
    buildSegments();
    buildArcLengthTable();
}

void Spline2D::setControlPoints(const std::vector<Vec2>& controlPoints, bool isClosed, SplineKnotMode mode)
{
    mControlPoints = controlPoints;
    mIsClosed = isClosed;
    mKnotMode = mode;
    buildSegments();
    buildArcLengthTable();
}

void Spline2D::clear() noexcept
{
    mControlPoints.clear();
    mSegments.clear();
    mCumulativeLengths.clear();
    mTotalLength = 0.0;
    mIsClosed = false;
}

std::size_t Spline2D::getPointCount() const noexcept
{
    return mControlPoints.size();
}

std::size_t Spline2D::getSegmentCount() const noexcept
{
    return mSegments.size();
}

bool Spline2D::isClosed() const noexcept
{
    return mIsClosed;
}

SplineKnotMode Spline2D::getKnotMode() const noexcept
{
    return mKnotMode;
}

double Spline2D::getTotalLength() const noexcept
{
    return mTotalLength;
}

double Spline2D::computeAlpha() const noexcept
{
    switch (mKnotMode) {
    case SplineKnotMode::Uniform:
        return 0.0;
    case SplineKnotMode::Chordal:
        return 1.0;
    case SplineKnotMode::Centripetal:
    default:
        return 0.5;
    }
}

void Spline2D::buildSegments()
{
    mSegments.clear();
    const std::size_t n { mControlPoints.size() };
    if (n < 2) {
        return;
    }

    const double alpha { computeAlpha() };
    const auto knotInterval = [alpha](const Vec2& a, const Vec2& b) noexcept -> double {
        if (alpha == 0.0) {
            return 1.0;
        }
        const double d2 { (b - a).lengthSquared() };
        if (d2 <= (kMinKnotDelta * kMinKnotDelta)) {
            return kMinKnotDelta;
        }
        if (alpha == 0.5) {
            return std::max(std::sqrt(std::sqrt(d2)), kMinKnotDelta);
        }
        if (alpha == 1.0) {
            return std::max(std::sqrt(d2), kMinKnotDelta);
        }
        return std::max(std::pow(std::sqrt(d2), alpha), kMinKnotDelta);
    };

    if (mIsClosed && (n >= 3)) {
        mSegments.reserve(n);
        for (std::size_t i { 0 }; i < n; ++i) {
            const std::size_t idx0 { (i + n - 1) % n };
            const std::size_t idx1 { i };
            const std::size_t idx2 { (i + 1) % n };
            const std::size_t idx3 { (i + 2) % n };

            SegmentKnotData seg {};
            seg.p0 = mControlPoints[idx0];
            seg.p1 = mControlPoints[idx1];
            seg.p2 = mControlPoints[idx2];
            seg.p3 = mControlPoints[idx3];

            seg.t0 = 0.0;
            seg.t1 = seg.t0 + knotInterval(seg.p0, seg.p1);
            seg.t2 = seg.t1 + knotInterval(seg.p1, seg.p2);
            seg.t3 = seg.t2 + knotInterval(seg.p2, seg.p3);

            mSegments.push_back(seg);
        }
    } else {
        const std::size_t segmentCount { n - 1 };
        mSegments.reserve(segmentCount);
        for (std::size_t i { 0 }; i < segmentCount; ++i) {
            SegmentKnotData seg {};
            seg.p1 = mControlPoints[i];
            seg.p2 = mControlPoints[i + 1];

            if (i == 0) {
                seg.p0 = (2.0 * seg.p1) - seg.p2;
            } else {
                seg.p0 = mControlPoints[i - 1];
            }

            if (i + 1 == segmentCount) {
                seg.p3 = (2.0 * seg.p2) - seg.p1;
            } else {
                seg.p3 = mControlPoints[i + 2];
            }

            seg.t0 = 0.0;
            seg.t1 = seg.t0 + knotInterval(seg.p0, seg.p1);
            seg.t2 = seg.t1 + knotInterval(seg.p1, seg.p2);
            seg.t3 = seg.t2 + knotInterval(seg.p2, seg.p3);

            mSegments.push_back(seg);
        }
    }
}

double Spline2D::computeSegmentArcLength(std::size_t segIndex, double uEnd) const noexcept
{
    if (segIndex >= mSegments.size() || uEnd <= 0.0) {
        return 0.0;
    }

    const auto& seg { mSegments[segIndex] };
    const double clampedUEnd { std::clamp(uEnd, 0.0, 1.0) };
    double sum { 0.0 };

    for (std::size_t k { 0 }; k < 5; ++k) {
        const double uMapped { 0.5 * clampedUEnd * (kGaussNodes[k] + 1.0) };
        const auto eval = evaluatePolynomial(seg.p0, seg.p1, seg.p2, seg.p3, seg.t0, seg.t1, seg.t2, seg.t3, uMapped);
        sum += kGaussWeights[k] * eval.velocity.length();
    }

    return 0.5 * clampedUEnd * sum;
}

void Spline2D::buildArcLengthTable()
{
    mCumulativeLengths.clear();
    mTotalLength = 0.0;

    if (mSegments.empty()) {
        return;
    }

    mCumulativeLengths.reserve(mSegments.size() + 1);
    mCumulativeLengths.push_back(0.0);

    for (std::size_t i { 0 }; i < mSegments.size(); ++i) {
        mSegments[i].arcLength = computeSegmentArcLength(i, 1.0);
        mTotalLength += mSegments[i].arcLength;
        mCumulativeLengths.push_back(mTotalLength);
    }
}

Vec2 Spline2D::evaluate(double t) const noexcept
{
    if (mControlPoints.empty()) {
        return Vec2::zero();
    }
    if (mControlPoints.size() == 1 || mSegments.empty()) {
        return mControlPoints.front();
    }

    double evalT { t };
    const double segCount { static_cast<double>(mSegments.size()) };

    if (mIsClosed) {
        evalT = std::fmod(evalT, segCount);
        if (evalT < 0.0) {
            evalT += segCount;
        }
    } else {
        evalT = std::clamp(evalT, 0.0, segCount);
    }

    auto segIndex { static_cast<std::size_t>(std::floor(evalT)) };
    double u { evalT - static_cast<double>(segIndex) };

    if (segIndex >= mSegments.size()) {
        segIndex = mSegments.size() - 1;
        u = 1.0;
    }

    const auto& seg { mSegments[segIndex] };
    return evaluatePolynomial(seg.p0, seg.p1, seg.p2, seg.p3, seg.t0, seg.t1, seg.t2, seg.t3, u).position;
}

SplineSample2D Spline2D::evaluateSample(double t) const noexcept
{
    SplineSample2D sample {};
    if (mControlPoints.empty()) {
        return sample;
    }
    if (mControlPoints.size() == 1 || mSegments.empty()) {
        sample.point = mControlPoints.front();
        return sample;
    }

    double evalT { t };
    const double segCount { static_cast<double>(mSegments.size()) };

    if (mIsClosed) {
        evalT = std::fmod(evalT, segCount);
        if (evalT < 0.0) {
            evalT += segCount;
        }
    } else {
        evalT = std::clamp(evalT, 0.0, segCount);
    }

    auto segIndex { static_cast<std::size_t>(std::floor(evalT)) };
    double u { evalT - static_cast<double>(segIndex) };

    if (segIndex >= mSegments.size()) {
        segIndex = mSegments.size() - 1;
        u = 1.0;
    }

    const auto& seg { mSegments[segIndex] };
    const auto eval = evaluatePolynomial(seg.p0, seg.p1, seg.p2, seg.p3, seg.t0, seg.t1, seg.t2, seg.t3, u);

    sample.point = eval.position;
    sample.velocity = eval.velocity;
    sample.acceleration = eval.acceleration;

    const double speed { eval.velocity.length() };
    if (speed > kEpsilon) {
        sample.tangent = eval.velocity / speed;
        sample.normal = sample.tangent.perpendicular();
        sample.curvature = eval.velocity.cross(eval.acceleration) / (speed * speed * speed);
    } else {
        sample.tangent = Vec2::unitX();
        sample.normal = Vec2::unitY();
        sample.curvature = 0.0;
    }

    sample.distance = mCumulativeLengths[segIndex] + computeSegmentArcLength(segIndex, u);
    return sample;
}

double Spline2D::mapDistanceToParameter(double s) const noexcept
{
    if (mSegments.empty()) {
        return 0.0;
    }
    if (mTotalLength <= kEpsilon) {
        return 0.0;
    }

    double targetS { s };
    if (mIsClosed) {
        targetS = std::fmod(targetS, mTotalLength);
        if (targetS < 0.0) {
            targetS += mTotalLength;
        }
    } else {
        targetS = std::clamp(targetS, 0.0, mTotalLength);
    }

    const auto it = std::upper_bound(mCumulativeLengths.begin(), mCumulativeLengths.end(), targetS);
    std::size_t segIndex { 0 };
    if (it != mCumulativeLengths.begin()) {
        segIndex = static_cast<std::size_t>(std::distance(mCumulativeLengths.begin(), it) - 1);
    }
    if (segIndex >= mSegments.size()) {
        segIndex = mSegments.size() - 1;
    }

    const double segStartDist { mCumulativeLengths[segIndex] };
    const double deltaS { targetS - segStartDist };
    const double segLength { mSegments[segIndex].arcLength };

    if (segLength <= kEpsilon) {
        return static_cast<double>(segIndex);
    }

    double u { deltaS / segLength };
    u = std::clamp(u, 0.0, 1.0);

    const auto& seg { mSegments[segIndex] };
    for (int iter { 0 }; iter < 4; ++iter) {
        const double currentDist { computeSegmentArcLength(segIndex, u) };
        const double diff { currentDist - deltaS };
        if (std::abs(diff) < 1e-9) {
            break;
        }

        const auto eval = evaluatePolynomial(seg.p0, seg.p1, seg.p2, seg.p3, seg.t0, seg.t1, seg.t2, seg.t3, u);
        const double speed { eval.velocity.length() };
        if (speed < 1e-9) {
            break;
        }
        u -= diff / speed;
        u = std::clamp(u, 0.0, 1.0);
    }

    return static_cast<double>(segIndex) + u;
}

SplineSample2D Spline2D::evaluateAtDistance(double s) const noexcept
{
    const double t { mapDistanceToParameter(s) };
    return evaluateSample(t);
}

double Spline2D::getCurvature(double t) const noexcept
{
    return evaluateSample(t).curvature;
}

SplineClosestPoint Spline2D::findClosest(const Vec2& query) const noexcept
{
    SplineClosestPoint result {};
    if (mControlPoints.empty()) {
        return result;
    }
    if (mControlPoints.size() == 1 || mSegments.empty()) {
        result.point = mControlPoints.front();
        result.distance = (result.point - query).length();
        result.arcLength = 0.0;
        result.parameter = 0.0;
        result.signedOffset = 0.0;
        return result;
    }

    constexpr std::size_t kSubdivisionsPerSeg { 10 };
    double bestDistSq { std::numeric_limits<double>::infinity() };
    std::size_t bestSeg { 0 };
    double bestU { 0.0 };

    for (std::size_t segIdx { 0 }; segIdx < mSegments.size(); ++segIdx) {
        const auto& seg { mSegments[segIdx] };
        for (std::size_t step { 0 }; step <= kSubdivisionsPerSeg; ++step) {
            const double uSample { static_cast<double>(step) / static_cast<double>(kSubdivisionsPerSeg) };
            const auto eval
                = evaluatePolynomial(seg.p0, seg.p1, seg.p2, seg.p3, seg.t0, seg.t1, seg.t2, seg.t3, uSample);
            const double d2 { (eval.position - query).lengthSquared() };
            if (d2 < bestDistSq) {
                bestDistSq = d2;
                bestSeg = segIdx;
                bestU = uSample;
            }
        }
    }

    double a { std::max(0.0, bestU - 0.12) };
    double b { std::min(1.0, bestU + 0.12) };
    constexpr double kInvPhi { 0.61803398874989484820 };
    constexpr double kPhiComp { 1.0 - kInvPhi };

    double c { a + (kPhiComp * (b - a)) };
    double d { b - (kPhiComp * (b - a)) };

    const auto& seg { mSegments[bestSeg] };
    const auto evalDistSq = [&seg, &query](double uVal) noexcept -> double {
        const auto eval = evaluatePolynomial(seg.p0, seg.p1, seg.p2, seg.p3, seg.t0, seg.t1, seg.t2, seg.t3, uVal);
        return (eval.position - query).lengthSquared();
    };

    double fc { evalDistSq(c) };
    double fd { evalDistSq(d) };

    for (int iter { 0 }; iter < 24; ++iter) {
        if (fc < fd) {
            b = d;
            d = c;
            fd = fc;
            c = a + (kPhiComp * (b - a));
            fc = evalDistSq(c);
        } else {
            a = c;
            c = d;
            fc = fd;
            d = b - (kPhiComp * (b - a));
            fd = evalDistSq(d);
        }
    }

    double refinedU { 0.5 * (a + b) };

    for (int iter { 0 }; iter < 4; ++iter) {
        const auto eval = evaluatePolynomial(seg.p0, seg.p1, seg.p2, seg.p3, seg.t0, seg.t1, seg.t2, seg.t3, refinedU);
        const Vec2 diff { eval.position - query };
        const double g { diff.dot(eval.velocity) };
        const double gPrime { eval.velocity.lengthSquared() + diff.dot(eval.acceleration) };
        if (std::abs(gPrime) < 1e-9) {
            break;
        }
        refinedU -= (g / gPrime);
        refinedU = std::clamp(refinedU, 0.0, 1.0);
    }

    const double refinedT { static_cast<double>(bestSeg) + refinedU };
    const SplineSample2D sample { evaluateSample(refinedT) };

    result.point = sample.point;
    result.distance = (sample.point - query).length();
    result.arcLength = sample.distance;
    result.parameter = refinedT;
    result.signedOffset = (query - sample.point).dot(sample.normal);

    return result;
}

std::vector<Vec2> Spline2D::discretize(double stepSize) const
{
    std::vector<Vec2> points {};
    if (mControlPoints.empty()) {
        return points;
    }
    if (mControlPoints.size() == 1 || mSegments.empty() || mTotalLength <= kEpsilon) {
        points.push_back(mControlPoints.front());
        return points;
    }

    const double safeStep { std::max(stepSize, 0.05) };
    const auto count = static_cast<std::size_t>(std::max(2.0, std::ceil(mTotalLength / safeStep) + 1.0));
    points.reserve(count);

    if (mIsClosed) {
        const std::size_t closedCount { count - 1 };
        const double deltaDist { mTotalLength / static_cast<double>(closedCount) };
        for (std::size_t i { 0 }; i < closedCount; ++i) {
            points.push_back(evaluateAtDistance(static_cast<double>(i) * deltaDist).point);
        }
    } else {
        const double deltaDist { mTotalLength / static_cast<double>(count - 1) };
        for (std::size_t i { 0 }; i < count; ++i) {
            points.push_back(evaluateAtDistance(static_cast<double>(i) * deltaDist).point);
        }
    }

    return points;
}

} // namespace vehicle_sim::math
