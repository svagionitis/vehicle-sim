#pragma once

#include <cstddef>
#include <cstdint>

#include "libs/VehicleSimCore/math/Vec2.h"

namespace vehicle_sim::physics {

/// @brief Maximum number of rigid bodies recorded in a fixed-capacity snapshot.
constexpr std::size_t kMaxSnapshotBodies { 128U };

/// @brief Maximum number of joints recorded in a fixed-capacity snapshot.
constexpr std::size_t kMaxSnapshotJoints { 64U };

/// @brief Maximum number of terrain delta cells recorded in a fixed-capacity snapshot.
constexpr std::size_t kMaxSnapshotTerrainDeltas { 256U };

/// @struct RigidBodySnapshot2D
/// @brief Bit-exact state of an individual RigidBody2D instance.
struct RigidBodySnapshot2D {
    std::uint32_t bodyId { 0U };
    math::Vec2 position { 0.0, 0.0 };
    math::Vec2 linearVelocity { 0.0, 0.0 };
    double orientation { 0.0 };
    double angularVelocity { 0.0 };
    math::Vec2 forceAccumulator { 0.0, 0.0 };
    double torqueAccumulator { 0.0 };
    bool isStatic { false };
    bool isBullet { false };
};

/// @struct JointSnapshot2D
/// @brief Bit-exact state of an individual Joint2D constraint.
struct JointSnapshot2D {
    std::uint32_t jointId { 0U };
    math::Vec2 linearImpulse { 0.0, 0.0 };
    double angularImpulse { 0.0 };
    double motorImpulse { 0.0 };
    double limitImpulse { 0.0 };
    bool isBroken { false };
};

/// @struct TerrainDeltaSnapshot2D
/// @brief Bit-exact elevation modification on deformable terrain grid.
struct TerrainDeltaSnapshot2D {
    std::int32_t gridX { 0 };
    std::int32_t gridY { 0 };
    double plasticDelta { 0.0 };
    double elasticDelta { 0.0 };
};

/// @class SimulationSnapshot2D
/// @brief Complete, self-contained, fixed-capacity simulation snapshot.
class SimulationSnapshot2D {
public:
    std::uint32_t frameIndex { 0U };
    double timestamp { 0.0 };

    std::uint32_t bodyCount { 0U };
    std::uint32_t jointCount { 0U };
    std::uint32_t terrainDeltaCount { 0U };
    std::uint64_t stateChecksum { 0ULL };

    RigidBodySnapshot2D bodies[kMaxSnapshotBodies] {};
    JointSnapshot2D joints[kMaxSnapshotJoints] {};
    TerrainDeltaSnapshot2D terrainDeltas[kMaxSnapshotTerrainDeltas] {};

    /// @brief Computes 64-bit FNV-1a checksum of the active state payload.
    /// @return 64-bit deterministic hash of all body, joint, and terrain records.
    [[nodiscard]] std::uint64_t computeChecksum() const noexcept;

    /// @brief Verifies whether the snapshot internal state matches its checksum.
    /// @return True if state is structurally valid and untampered.
    [[nodiscard]] bool isValid() const noexcept;
};

/// @class SnapshotRingBuffer2D
/// @brief Fixed-capacity circular ring buffer for storing historical snapshots.
/// @tparam Capacity Number of historical frames preserved (default 64 frames = ~1 sec at 60 Hz).
template <std::size_t Capacity = 64U>
class SnapshotRingBuffer2D {
public:
    static_assert(Capacity > 0U, "Ring buffer capacity must be greater than zero");

    /// @brief Default constructor initializing ring indices.
    SnapshotRingBuffer2D() noexcept : mHead { 0U }, mSize { 0U } {}

    /// @brief Pushes a snapshot into the ring buffer, overwriting the oldest entry.
    /// @param[in] snapshot Snapshot to record.
    void push(const SimulationSnapshot2D& snapshot) noexcept {
        mBuffer[mHead] = snapshot;
        mHead = (mHead + 1U) % Capacity;
        if (mSize < Capacity) {
            ++mSize;
        }
    }

    /// @brief Retrieves snapshot recorded for a specific frame index.
    /// @param[in] frame Target simulation frame index.
    /// @return Pointer to snapshot if present in ring, nullptr otherwise.
    [[nodiscard]] const SimulationSnapshot2D* get(std::uint32_t frame) const noexcept {
        for (std::size_t i { 0U }; i < mSize; ++i) {
            if (mBuffer[i].frameIndex == frame) {
                return &mBuffer[i];
            }
        }
        return nullptr;
    }

    /// @brief Retrieves the latest recorded snapshot.
    /// @return Pointer to latest snapshot, or nullptr if empty.
    [[nodiscard]] const SimulationSnapshot2D* getLatest() const noexcept {
        if (mSize == 0U) {
            return nullptr;
        }
        const std::size_t latestIdx { (mHead == 0U) ? (Capacity - 1U) : (mHead - 1U) };
        return &mBuffer[latestIdx];
    }

    /// @brief Returns the number of snapshots currently stored.
    /// @return Count of valid snapshots.
    [[nodiscard]] std::size_t size() const noexcept {
        return mSize;
    }

    /// @brief Returns the capacity of the ring buffer.
    /// @return Maximum snapshot storage capacity.
    [[nodiscard]] constexpr std::size_t capacity() const noexcept {
        return Capacity;
    }

    /// @brief Queries whether the ring buffer is empty.
    /// @return True if no snapshots are stored.
    [[nodiscard]] bool empty() const noexcept {
        return mSize == 0U;
    }

    /// @brief Clears the ring buffer.
    void clear() noexcept {
        mHead = 0U;
        mSize = 0U;
    }

private:
    SimulationSnapshot2D mBuffer[Capacity] {};
    std::size_t mHead { 0U };
    std::size_t mSize { 0U };
};

} // namespace vehicle_sim::physics
