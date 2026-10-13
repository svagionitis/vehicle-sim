#include "libs/VehicleSimCore/telemetry/TelemetryRingBuffer2D.h"

#include <algorithm>
#include <cmath>

namespace vehicle_sim::telemetry {

namespace {
    const TelemetryFrame2D kEmptyFrame {};
} // namespace

TelemetryRingBuffer2D::TelemetryRingBuffer2D(std::size_t capacity)
    : m_capacity { std::max<std::size_t>(1U, capacity) }
{
    m_storage.resize(m_capacity);
}

void TelemetryRingBuffer2D::push(const TelemetryFrame2D& frame) noexcept
{
    m_storage[m_head] = frame;
    m_head = (m_head + 1U) % m_capacity;

    if (m_count < m_capacity) {
        ++m_count;
    } else {
        m_tail = (m_tail + 1U) % m_capacity;
    }
}

void TelemetryRingBuffer2D::clear() noexcept
{
    m_head = 0;
    m_tail = 0;
    m_count = 0;
}

bool TelemetryRingBuffer2D::isEmpty() const noexcept
{
    return m_count == 0;
}

bool TelemetryRingBuffer2D::isFull() const noexcept
{
    return m_count == m_capacity;
}

std::size_t TelemetryRingBuffer2D::getCount() const noexcept
{
    return m_count;
}

std::size_t TelemetryRingBuffer2D::getCapacity() const noexcept
{
    return m_capacity;
}

const TelemetryFrame2D& TelemetryRingBuffer2D::getLatest() const noexcept
{
    if (m_count == 0) {
        return kEmptyFrame;
    }
    const std::size_t latestIdx { (m_head + m_capacity - 1U) % m_capacity };
    return m_storage[latestIdx];
}

const TelemetryFrame2D& TelemetryRingBuffer2D::getOldest() const noexcept
{
    if (m_count == 0) {
        return kEmptyFrame;
    }
    return m_storage[m_tail];
}

const TelemetryFrame2D& TelemetryRingBuffer2D::getFrame(std::size_t chronologicalIndex) const noexcept
{
    if (m_count == 0) {
        return kEmptyFrame;
    }
    const std::size_t clampedIndex { std::min(chronologicalIndex, m_count - 1U) };
    const std::size_t slot { (m_tail + clampedIndex) % m_capacity };
    return m_storage[slot];
}

const TelemetryFrame2D& TelemetryRingBuffer2D::getPastFrame(std::size_t reverseIndex) const noexcept
{
    if (m_count == 0) {
        return kEmptyFrame;
    }
    const std::size_t clampedIndex { std::min(reverseIndex, m_count - 1U) };
    const std::size_t slot { (m_head + m_capacity - 1U - clampedIndex) % m_capacity };
    return m_storage[slot];
}

void TelemetryRingBuffer2D::getPeakGForces(double& maxLongG, double& maxLatG) const noexcept
{
    maxLongG = 0.0;
    maxLatG = 0.0;

    for (std::size_t i { 0 }; i < m_count; ++i) {
        const std::size_t slot { (m_tail + i) % m_capacity };
        const double absLongG { std::abs(m_storage[slot].longitudinalG) };
        const double absLatG { std::abs(m_storage[slot].lateralG) };
        if (absLongG > maxLongG) {
            maxLongG = absLongG;
        }
        if (absLatG > maxLatG) {
            maxLatG = absLatG;
        }
    }
}

} // namespace vehicle_sim::telemetry
