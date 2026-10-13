#include "libs/VehicleSimCore/telemetry/KlvTelemetryExporter.h"

#include <algorithm>
#include <cmath>

namespace vehicle_sim::telemetry {

namespace {
    constexpr double kPi { 3.14159265358979323846 };
    constexpr double kMetersPerDegreeLat { 111132.95 };

    const std::array<std::uint8_t, 16> kUasDatalinkKey { 0x06, 0x0E, 0x2B, 0x34, 0x02, 0x0B, 0x01, 0x01, 0x0E, 0x01,
        0x03, 0x01, 0x01, 0x00, 0x00, 0x00 };
} // namespace

KlvTelemetryExporter::KlvTelemetryExporter(const KlvGeoReference& geoRef) noexcept
    : m_geoRef { geoRef }
{
}

std::uint16_t KlvTelemetryExporter::computeCrc(const std::uint8_t* data, std::size_t length) noexcept
{
    if (data == nullptr || length == 0) {
        return 0;
    }

    std::uint16_t crc { 0x0000 };
    for (std::size_t i { 0 }; i < length; ++i) {
        crc ^= static_cast<std::uint16_t>(static_cast<std::uint16_t>(data[i]) << 8U);
        for (int b { 0 }; b < 8; ++b) {
            if ((crc & 0x8000U) != 0U) {
                crc = static_cast<std::uint16_t>((crc << 1U) ^ 0x1021U);
            } else {
                crc = static_cast<std::uint16_t>(crc << 1U);
            }
        }
    }
    return crc;
}

bool KlvTelemetryExporter::encodePacket(const TelemetryFrame2D& frame, std::vector<std::uint8_t>& outBytes) const
{
    std::vector<std::uint8_t> payload {};
    payload.reserve(128);

    // Tag 65: UAS Datalink LS Version (1 byte: Version 12)
    payload.push_back(65);
    payload.push_back(1);
    payload.push_back(12);

    // Tag 2: Precision Time Stamp (8 bytes microseconds)
    const auto usec { static_cast<std::uint64_t>(std::max(0.0, frame.timestamp) * 1e6) };
    payload.push_back(2);
    payload.push_back(8);
    for (int i { 7 }; i >= 0; --i) {
        payload.push_back(static_cast<std::uint8_t>((usec >> (static_cast<unsigned int>(i) * 8U)) & 0xFFU));
    }

    // Tag 5: Platform Heading Angle (2 bytes uint, [0, 360) deg -> [0, 65535])
    double headingDeg { std::fmod(frame.heading * (180.0 / kPi), 360.0) };
    if (headingDeg < 0.0) {
        headingDeg += 360.0;
    }
    const auto headingVal { static_cast<std::uint16_t>(
        std::clamp(std::round((headingDeg / 360.0) * 65535.0), 0.0, 65535.0)) };
    payload.push_back(5);
    payload.push_back(2);
    payload.push_back(static_cast<std::uint8_t>((headingVal >> 8U) & 0xFFU));
    payload.push_back(static_cast<std::uint8_t>(headingVal & 0xFFU));

    // Tag 8: Platform Ground Speed (1 byte uint, [0, 255] m/s)
    const auto speedVal { static_cast<std::uint8_t>(std::clamp(std::round(std::abs(frame.forwardSpeed)), 0.0, 255.0)) };
    payload.push_back(8);
    payload.push_back(1);
    payload.push_back(speedVal);

    // Tag 13: Sensor Latitude (4 bytes int, [-90, 90] deg -> [-2147483647, 2147483647])
    double latDeg { m_geoRef.originLatitudeDeg + (frame.position.y / kMetersPerDegreeLat) };
    latDeg = std::clamp(latDeg, -90.0, 90.0);
    const auto latVal { static_cast<std::int32_t>(std::round((latDeg / 90.0) * 2147483647.0)) };
    payload.push_back(13);
    payload.push_back(4);
    for (int i { 3 }; i >= 0; --i) {
        payload.push_back(static_cast<std::uint8_t>(
            (static_cast<std::uint32_t>(latVal) >> (static_cast<unsigned int>(i) * 8U)) & 0xFFU));
    }

    // Tag 14: Sensor Longitude (4 bytes int, [-180, 180] deg -> [-2147483647, 2147483647])
    const double radLat { m_geoRef.originLatitudeDeg * (kPi / 180.0) };
    const double metersPerDegLon { kMetersPerDegreeLat * std::max(0.01, std::cos(radLat)) };
    double lonDeg { m_geoRef.originLongitudeDeg + (frame.position.x / metersPerDegLon) };
    lonDeg = std::clamp(lonDeg, -180.0, 180.0);
    const auto lonVal { static_cast<std::int32_t>(std::round((lonDeg / 180.0) * 2147483647.0)) };
    payload.push_back(14);
    payload.push_back(4);
    for (int i { 3 }; i >= 0; --i) {
        payload.push_back(static_cast<std::uint8_t>(
            (static_cast<std::uint32_t>(lonVal) >> (static_cast<unsigned int>(i) * 8U)) & 0xFFU));
    }

    // Tag 15: Sensor True Altitude (2 bytes uint, [-900, 19000] m -> [0, 65535])
    const double altM { m_geoRef.originAltitudeMeters + frame.altitude };
    const auto altVal { static_cast<std::uint16_t>(
        std::clamp(std::round(((altM + 900.0) / 19900.0) * 65535.0), 0.0, 65535.0)) };
    payload.push_back(15);
    payload.push_back(2);
    payload.push_back(static_cast<std::uint8_t>((altVal >> 8U) & 0xFFU));
    payload.push_back(static_cast<std::uint8_t>(altVal & 0xFFU));

    // Tag 1: Checksum placeholder (Tag 1, Length 2, Value 2 bytes)
    payload.push_back(1);
    payload.push_back(2);
    payload.push_back(0);
    payload.push_back(0);

    // BER Length Encoding
    const std::size_t payloadLen { payload.size() };
    std::vector<std::uint8_t> berLen {};
    if (payloadLen < 128U) {
        berLen.push_back(static_cast<std::uint8_t>(payloadLen));
    } else if (payloadLen <= 255U) {
        berLen.push_back(0x81U);
        berLen.push_back(static_cast<std::uint8_t>(payloadLen));
    } else {
        berLen.push_back(0x82U);
        berLen.push_back(static_cast<std::uint8_t>((payloadLen >> 8U) & 0xFFU));
        berLen.push_back(static_cast<std::uint8_t>(payloadLen & 0xFFU));
    }

    // Assemble Full Packet
    outBytes.clear();
    outBytes.reserve(kUasDatalinkKey.size() + berLen.size() + payloadLen);
    outBytes.insert(outBytes.end(), kUasDatalinkKey.begin(), kUasDatalinkKey.end());
    outBytes.insert(outBytes.end(), berLen.begin(), berLen.end());
    outBytes.insert(outBytes.end(), payload.begin(), payload.end());

    // Calculate Tag 1 CRC-16 over all bytes up to Tag 1 length byte
    const std::size_t crcLength { outBytes.size() - 2U };
    const std::uint16_t crc { computeCrc(outBytes.data(), crcLength) };

    outBytes[outBytes.size() - 2U] = static_cast<std::uint8_t>((crc >> 8U) & 0xFFU);
    outBytes[outBytes.size() - 1U] = static_cast<std::uint8_t>(crc & 0xFFU);

    return true;
}

} // namespace vehicle_sim::telemetry
