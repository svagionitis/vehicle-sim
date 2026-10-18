#include "SimulationSnapshot2D.h"

namespace vehicle_sim::physics {

namespace {

constexpr std::uint64_t kFnvOffsetBasis { 0xcbf29ce484222325ULL };
constexpr std::uint64_t kFnvPrime { 0x100000001b3ULL };

inline void hashBytes(std::uint64_t& hash, const void* data, std::size_t size) noexcept
{
    const auto* bytes = static_cast<const unsigned char*>(data);
    for (std::size_t i { 0U }; i < size; ++i) {
        hash ^= static_cast<std::uint64_t>(bytes[i]);
        hash *= kFnvPrime;
    }
}

} // namespace

std::uint64_t SimulationSnapshot2D::computeChecksum() const noexcept
{
    std::uint64_t hash { kFnvOffsetBasis };
    hashBytes(hash, &frameIndex, sizeof(frameIndex));
    hashBytes(hash, &timestamp, sizeof(timestamp));
    hashBytes(hash, &bodyCount, sizeof(bodyCount));
    hashBytes(hash, &jointCount, sizeof(jointCount));
    hashBytes(hash, &terrainDeltaCount, sizeof(terrainDeltaCount));

    const std::size_t validBodies { (bodyCount < kMaxSnapshotBodies) ? bodyCount : kMaxSnapshotBodies };
    for (std::size_t i { 0U }; i < validBodies; ++i) {
        hashBytes(hash, &bodies[i], sizeof(RigidBodySnapshot2D));
    }

    const std::size_t validJoints { (jointCount < kMaxSnapshotJoints) ? jointCount : kMaxSnapshotJoints };
    for (std::size_t i { 0U }; i < validJoints; ++i) {
        hashBytes(hash, &joints[i], sizeof(JointSnapshot2D));
    }

    const std::size_t validTerrain {
        (terrainDeltaCount < kMaxSnapshotTerrainDeltas) ? terrainDeltaCount : kMaxSnapshotTerrainDeltas
    };
    for (std::size_t i { 0U }; i < validTerrain; ++i) {
        hashBytes(hash, &terrainDeltas[i], sizeof(TerrainDeltaSnapshot2D));
    }

    return hash;
}

bool SimulationSnapshot2D::isValid() const noexcept
{
    if (bodyCount > kMaxSnapshotBodies || jointCount > kMaxSnapshotJoints
        || terrainDeltaCount > kMaxSnapshotTerrainDeltas) {
        return false;
    }
    return computeChecksum() == stateChecksum;
}

} // namespace vehicle_sim::physics
