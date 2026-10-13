#pragma once

#include <iosfwd>
#include <string>

#include "libs/VehicleSimCore/telemetry/TelemetryRingBuffer2D.h"

namespace vehicle_sim::telemetry {

/// @class CsvTelemetryExporter
/// @brief Serializes telemetry ring buffers into MoTeC and PlotJuggler compatible CSV.
class CsvTelemetryExporter {
public:
    /// @brief Default constructor.
    CsvTelemetryExporter() noexcept = default;

    /// @brief Writes full CSV contents from ring buffer to output stream.
    /// @param[in] buffer Source telemetry ring buffer.
    /// @param[out] output Stream to receive serialized CSV text.
    /// @return True on successful write, false on stream error.
    [[nodiscard]] bool writeCsv(const TelemetryRingBuffer2D& buffer, std::ostream& output) const;

    /// @brief Formats the standard CSV channel header line.
    /// @return Formatted header string with channel names and units.
    [[nodiscard]] std::string getHeader() const;

    /// @brief Formats an individual telemetry frame into a single CSV row.
    /// @param[in] frame Telemetry frame to serialize.
    /// @return Comma-separated field string.
    [[nodiscard]] std::string formatRow(const TelemetryFrame2D& frame) const;
};

} // namespace vehicle_sim::telemetry
