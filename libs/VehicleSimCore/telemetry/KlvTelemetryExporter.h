#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include "libs/VehicleSimCore/telemetry/TelemetryFrame2D.h"

namespace vehicle_sim::telemetry {

/// @struct KlvGeoReference
/// @brief Geodetic anchor mapping local simulation 2D coordinates to WGS-84 coordinates.
struct KlvGeoReference {
    double originLatitudeDeg { 37.7749 }; ///< Local origin latitude in degrees.
    double originLongitudeDeg { -122.4194 }; ///< Local origin longitude in degrees.
    double originAltitudeMeters { 100.0 }; ///< Local origin altitude in meters.
};

/// @class KlvTelemetryExporter
/// @brief Encodes telemetry frames into MISB ST 0601 / SMPTE 336M byte packets.
class KlvTelemetryExporter {
public:
    /// @brief Default constructor with default WGS-84 origin.
    KlvTelemetryExporter() noexcept = default;

    /// @brief Parameterized constructor with geodetic anchor.
    /// @param[in] geoRef Local origin anchor coordinates.
    explicit KlvTelemetryExporter(const KlvGeoReference& geoRef) noexcept;

    /// @brief Encodes a telemetry frame into a self-contained MISB ST 0601 packet.
    /// @param[in] frame Telemetry frame sample to encode.
    /// @param[out] outBytes Output byte buffer receiving the KLV packet.
    /// @return True if encoding succeeded, false on buffer failure.
    [[nodiscard]] bool encodePacket(const TelemetryFrame2D& frame, std::vector<std::uint8_t>& outBytes) const;

    /// @brief Computes 16-bit CRC-CCITT over packet bytes according to MISB ST 0601 Tag 1.
    /// @param[in] data Buffer pointer.
    /// @param[in] length Number of bytes to compute over.
    /// @return 16-bit CRC checksum.
    [[nodiscard]] static std::uint16_t computeCrc(const std::uint8_t* data, std::size_t length) noexcept;

private:
    KlvGeoReference m_geoRef {};
};

} // namespace vehicle_sim::telemetry
