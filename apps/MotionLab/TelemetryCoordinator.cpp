#include "apps/MotionLab/TelemetryCoordinator.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <ctime>
#include <iomanip>
#include <sstream>

namespace motion_lab {

TelemetryCoordinator::TelemetryCoordinator()
    : m_buffer(1200U)
    , m_sampler(0.25)
{
}

TelemetryCoordinator::~TelemetryCoordinator() noexcept
{
    if (m_recordStream.is_open()) {
        m_recordStream.flush();
        m_recordStream.close();
    }
}

void TelemetryCoordinator::reset() noexcept
{
    m_buffer.clear();
    m_sampler.reset();

    m_oscilloscope = OscilloscopeWindow { };
    m_ggTrail = { };
    m_ggHead = 0U;
    m_ggCount = 0U;
    m_currentLoads = DynamicWheelLoads { };

    m_peakLongG = 0.0;
    m_peakLatG = 0.0;
    m_lastSurfaceMu = 1.0;
    m_lastSpeedMps = 0.0;
    m_lastYawRate = 0.0;

    if (m_recordStream.is_open()) {
        m_recordStream.flush();
        m_recordStream.close();
    }
    m_isRecording = false;
    m_recordStartTime = 0.0;
    m_recordDuration = 0.0;
    m_recordFrames = 0U;
    m_recordFilename.clear();
}

void TelemetryCoordinator::computeLoads(
    double mass, double wheelbase, double trackWidth, double cgHeight, double longG, double latG) noexcept
{
    constexpr double kGravity { 9.80665 };
    const double safeMass { std::max(50.0, mass) };
    const double safeWb { std::max(0.5, wheelbase) };
    const double safeTw { std::max(0.5, trackWidth) };
    const double safeCgH { std::max(0.1, cgHeight) };

    const double totalWeight { safeMass * kGravity };
    const double staticFront { totalWeight * 0.5 };
    const double staticRear { totalWeight * 0.5 };

    m_currentLoads.staticFrontWheel = staticFront * 0.5;
    m_currentLoads.staticRearWheel = staticRear * 0.5;

    // Longitudinal acceleration acts along vehicle axis
    const double ax { longG * kGravity };
    const double deltaLong { (safeMass * ax * safeCgH) / safeWb };

    const double frontAxle { staticFront - deltaLong };
    const double rearAxle { staticRear + deltaLong };

    // Lateral cornering acceleration roll distribution: 52% front, 48% rear
    const double ay { latG * kGravity };
    constexpr double kFrontRollStiffnessRatio { 0.52 };
    constexpr double kRearRollStiffnessRatio { 0.48 };

    const double deltaLatFront { (kFrontRollStiffnessRatio * safeMass * ay * safeCgH) / safeTw };
    const double deltaLatRear { (kRearRollStiffnessRatio * safeMass * ay * safeCgH) / safeTw };

    // Individual 4-corner wheel reactions with zero lift-off clamping
    m_currentLoads.frontLeftLoad = std::max(0.0, (frontAxle * 0.5) - deltaLatFront);
    m_currentLoads.frontRightLoad = std::max(0.0, (frontAxle * 0.5) + deltaLatFront);
    m_currentLoads.rearLeftLoad = std::max(0.0, (rearAxle * 0.5) - deltaLatRear);
    m_currentLoads.rearRightLoad = std::max(0.0, (rearAxle * 0.5) + deltaLatRear);
}

void TelemetryCoordinator::update(const vehicle_sim::physics::RigidBody2D& body, double mass, double wheelbase,
    double trackWidth, double cgHeight, double surfaceMu, double frontSlip, double rearSlip, double simTime,
    double dt) noexcept
{
    m_lastSurfaceMu = surfaceMu;
    const vehicle_sim::telemetry::TelemetryFrame2D frame { m_sampler.sample(body, nullptr, simTime, dt) };

    m_lastSpeedMps = frame.forwardSpeed;
    m_lastYawRate = frame.angularVelocity;

    // Update peak acceleration statistics
    m_peakLongG = std::max(m_peakLongG, std::abs(frame.longitudinalG));
    m_peakLatG = std::max(m_peakLatG, std::abs(frame.lateralG));

    // Compute dynamic 4-corner normal wheel loads
    computeLoads(mass, wheelbase, trackWidth, cgHeight, frame.longitudinalG, frame.lateralG);

    // Push into continuous history buffer
    m_buffer.push(frame);

    // Update G-G trail circular buffer
    m_ggTrail[m_ggHead] = vehicle_sim::math::Vec2 { frame.longitudinalG, frame.lateralG };
    m_ggHead = (m_ggHead + 1U) % kGgTrailSize;
    if (m_ggCount < kGgTrailSize) {
        ++m_ggCount;
    }

    // Update rolling oscilloscope strip chart buffers
    constexpr float kRadToDeg { 57.2957795f };
    const std::size_t oscHead { m_oscilloscope.headIndex };
    m_oscilloscope.speedHistory[oscHead] = static_cast<float>(frame.forwardSpeed);
    m_oscilloscope.frontSlipHistory[oscHead] = static_cast<float>(frontSlip) * kRadToDeg;
    m_oscilloscope.rearSlipHistory[oscHead] = static_cast<float>(rearSlip) * kRadToDeg;
    m_oscilloscope.yawRateHistory[oscHead] = static_cast<float>(frame.angularVelocity) * kRadToDeg;

    m_oscilloscope.headIndex = (oscHead + 1U) % OscilloscopeWindow::kHistoryCapacity;
    if (m_oscilloscope.sampleCount < OscilloscopeWindow::kHistoryCapacity) {
        ++m_oscilloscope.sampleCount;
    }

    // Stream to active disk session if recording
    if (m_isRecording) {
        if (m_recordFrames == 0U) {
            m_recordStartTime = simTime;
        }
        ++m_recordFrames;
        m_recordDuration = std::max(0.0, simTime - m_recordStartTime);

        if (m_recordStream.is_open()) {
            m_recordStream << m_exporter.formatRow(frame) << "\n";
        }
    }
}

std::string TelemetryCoordinator::makeFilename() noexcept
{
    const auto now { std::chrono::system_clock::now() };
    const std::time_t timeT { std::chrono::system_clock::to_time_t(now) };
    std::tm timeBuf { };

#if defined(_MSC_VER) || defined(__STDC_LIB_EXT1__)
    localtime_s(&timeBuf, &timeT);
#else
    localtime_r(&timeT, &timeBuf);
#endif

    std::ostringstream ss { };
    ss << "telemetry_run_" << std::setfill('0') << std::setw(4) << (timeBuf.tm_year + 1900) << std::setw(2)
       << (timeBuf.tm_mon + 1) << std::setw(2) << timeBuf.tm_mday << "_" << std::setw(2) << timeBuf.tm_hour
       << std::setw(2) << timeBuf.tm_min << std::setw(2) << timeBuf.tm_sec << ".csv";

    return ss.str();
}

bool TelemetryCoordinator::toggleRecording() noexcept
{
    if (m_isRecording) {
        m_isRecording = false;
        if (m_recordStream.is_open()) {
            m_recordStream.flush();
            m_recordStream.close();
        }
        return false;
    }

    m_isRecording = true;
    m_recordStartTime = 0.0;
    m_recordDuration = 0.0;
    m_recordFrames = 0U;
    m_recordFilename = makeFilename();

    m_recordStream.open(m_recordFilename);
    if (m_recordStream.is_open()) {
        m_recordStream << m_exporter.getHeader() << "\n";
    }

    return true;
}

bool TelemetryCoordinator::isRecording() const noexcept
{
    return m_isRecording;
}

double TelemetryCoordinator::getRecordDuration() const noexcept
{
    return m_recordDuration;
}

std::uint64_t TelemetryCoordinator::getRecordFrames() const noexcept
{
    return m_recordFrames;
}

const std::string& TelemetryCoordinator::getRecordFile() const noexcept
{
    return m_recordFilename;
}

void TelemetryCoordinator::populateSnapshot(ExtendedTelemetrySnapshot& outSnapshot) const noexcept
{
    if (!m_buffer.isEmpty()) {
        const vehicle_sim::telemetry::TelemetryFrame2D& latest { m_buffer.getLatest() };
        outSnapshot.speedMps = latest.forwardSpeed;
        outSnapshot.speedMph = latest.forwardSpeed * 2.23694;
        outSnapshot.angularVelocity = latest.angularVelocity;
        outSnapshot.longitudinalG = latest.longitudinalG;
        outSnapshot.lateralG = latest.lateralG;
    } else {
        outSnapshot.speedMps = m_lastSpeedMps;
        outSnapshot.speedMph = m_lastSpeedMps * 2.23694;
        outSnapshot.angularVelocity = m_lastYawRate;
        outSnapshot.longitudinalG = 0.0;
        outSnapshot.lateralG = 0.0;
    }

    outSnapshot.peakLongG = m_peakLongG;
    outSnapshot.peakLatG = m_peakLatG;
    outSnapshot.surfaceFriction = m_lastSurfaceMu;
    outSnapshot.wheelLoads = m_currentLoads;

    outSnapshot.ggTrail = m_ggTrail;
    outSnapshot.ggTrailCount = m_ggCount;
    outSnapshot.ggTrailHead = m_ggHead;

    outSnapshot.stripCharts = m_oscilloscope;

    outSnapshot.isRecording = m_isRecording;
    outSnapshot.recordingDuration = m_recordDuration;
    outSnapshot.recordedFrames = m_recordFrames;
}

const vehicle_sim::telemetry::TelemetryRingBuffer2D& TelemetryCoordinator::getBuffer() const noexcept
{
    return m_buffer;
}

} // namespace motion_lab
