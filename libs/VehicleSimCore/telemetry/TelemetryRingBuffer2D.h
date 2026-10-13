#pragma once

#include <cstddef>
#include <vector>

#include "libs/VehicleSimCore/telemetry/TelemetryFrame2D.h"

namespace vehicle_sim::telemetry {

/// @class TelemetryRingBuffer2D
/// @brief Deterministic, fixed-capacity circular buffer storing high-rate telemetry.
/// @details Pre-allocates all memory during initialization to guarantee zero dynamic
/// heap allocations during simulation steps. Supports constant-time frame push,
/// reverse historical indexing, and time-range query extraction.
class TelemetryRingBuffer2D {
public:
    /// @brief Parameterized constructor allocating fixed capacity.
    /// @param[in] capacity Maximum number of telemetry frames retained.
    explicit TelemetryRingBuffer2D(std::size_t capacity);

    /// @brief Destructor.
    ~TelemetryRingBuffer2D() noexcept = default;

    /// @brief Appends a new telemetry frame into the circular buffer.
    /// @details Overwrites the oldest sample if the buffer is at capacity.
    /// @param[in] frame Telemetry frame sample to record.
    void push(const TelemetryFrame2D& frame) noexcept;

    /// @brief Clears all recorded telemetry frames.
    void clear() noexcept;

    /// @brief Queries whether the buffer contains zero frames.
    /// @return True if empty, false otherwise.
    [[nodiscard]] bool isEmpty() const noexcept;

    /// @brief Queries whether the buffer has reached maximum capacity.
    /// @return True if frame count equals buffer capacity.
    [[nodiscard]] bool isFull() const noexcept;

    /// @brief Gets the number of valid frames currently stored.
    /// @return Count of active frames.
    [[nodiscard]] std::size_t getCount() const noexcept;

    /// @brief Gets the maximum frame capacity of the buffer.
    /// @return Maximum capacity.
    [[nodiscard]] std::size_t getCapacity() const noexcept;

    /// @brief Accesses the most recently recorded telemetry frame.
    /// @return Const reference to latest frame.
    [[nodiscard]] const TelemetryFrame2D& getLatest() const noexcept;

    /// @brief Accesses the oldest recorded telemetry frame currently held.
    /// @return Const reference to oldest frame.
    [[nodiscard]] const TelemetryFrame2D& getOldest() const noexcept;

    /// @brief Accesses frame by chronological offset from oldest sample.
    /// @param[in] chronologicalIndex Offset from oldest (0 = oldest, count-1 = newest).
    /// @return Const reference to requested frame.
    [[nodiscard]] const TelemetryFrame2D& getFrame(std::size_t chronologicalIndex) const noexcept;

    /// @brief Accesses frame by reverse history from newest sample.
    /// @param[in] reverseIndex Offset into past (0 = newest, count-1 = oldest).
    /// @return Const reference to requested frame.
    [[nodiscard]] const TelemetryFrame2D& getPastFrame(std::size_t reverseIndex) const noexcept;

    /// @brief Computes peak longitudinal and lateral G forces in buffer.
    /// @param[out] maxLongG Maximum absolute longitudinal G recorded.
    /// @param[out] maxLatG Maximum absolute lateral G recorded.
    void getPeakGForces(double& maxLongG, double& maxLatG) const noexcept;

private:
    std::vector<TelemetryFrame2D> m_storage {};
    std::size_t m_capacity { 0 };
    std::size_t m_head { 0 };
    std::size_t m_tail { 0 };
    std::size_t m_count { 0 };
};

} // namespace vehicle_sim::telemetry
