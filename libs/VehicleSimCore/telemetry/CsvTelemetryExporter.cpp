#include "libs/VehicleSimCore/telemetry/CsvTelemetryExporter.h"

#include <iomanip>
#include <ostream>
#include <sstream>

namespace vehicle_sim::telemetry {

std::string CsvTelemetryExporter::getHeader() const
{
    return "Time,Tick,PosX,PosY,Speed,Heading,YawRate,AccelX,AccelY,LongG,LatG,"
           "Throttle,Steer,Brake,Handbrake,RPM,Gear,Torque,WheelCount,"
           "FL_SlipAngle,FL_NormalForce,FR_SlipAngle,FR_NormalForce,"
           "RL_SlipAngle,RL_NormalForce,RR_SlipAngle,RR_NormalForce,Altitude,Airborne";
}

std::string CsvTelemetryExporter::formatRow(const TelemetryFrame2D& frame) const
{
    std::ostringstream ss {};
    ss << std::fixed << std::setprecision(4);

    ss << frame.timestamp << ',' << frame.tickNumber << ',' << frame.position.x << ',' << frame.position.y << ','
       << frame.forwardSpeed << ',' << frame.heading << ',' << frame.angularVelocity << ',' << frame.acceleration.x
       << ',' << frame.acceleration.y << ',' << frame.longitudinalG << ',' << frame.lateralG << ','
       << frame.throttleInput << ',' << frame.steeringInput << ',' << frame.brakeInput << ',' << frame.handbrakeInput
       << ',' << frame.engineRpm << ',' << frame.activeGear << ',' << frame.driveTorque << ',' << frame.wheelCount
       << ',';

    for (std::size_t i { 0 }; i < TelemetryFrame2D::kMaxWheels; ++i) {
        ss << frame.wheels[i].slipAngle << ',' << frame.wheels[i].normalForce << ',';
    }

    ss << frame.altitude << ',' << (frame.isAirborne ? 1 : 0);

    return ss.str();
}

bool CsvTelemetryExporter::writeCsv(const TelemetryRingBuffer2D& buffer, std::ostream& output) const
{
    if (!output.good()) {
        return false;
    }

    output << getHeader() << '\n';

    const std::size_t count { buffer.getCount() };
    for (std::size_t i { 0 }; i < count; ++i) {
        const TelemetryFrame2D& frame { buffer.getFrame(i) };
        output << formatRow(frame) << '\n';
    }

    return output.good();
}

} // namespace vehicle_sim::telemetry
