#include "libs/VehicleSimCore/telemetry/TelemetryFrame2D.h"

namespace vehicle_sim::telemetry {

void TelemetryFrame2D::reset() noexcept
{
    tickNumber = 0;
    timestamp = 0.0;

    position = math::Vec2 {};
    velocity = math::Vec2 {};
    forwardSpeed = 0.0;
    heading = 0.0;
    angularVelocity = 0.0;

    acceleration = math::Vec2 {};
    longitudinalG = 0.0;
    lateralG = 0.0;

    throttleInput = 0.0;
    steeringInput = 0.0;
    brakeInput = 0.0;
    handbrakeInput = 0.0;

    engineRpm = 0.0;
    activeGear = 0;
    driveTorque = 0.0;

    wheelCount = 0;
    wheels.fill(WheelSample2D {});

    altitude = 0.0;
    verticalSpeed = 0.0;
    isAirborne = false;

    auxChannels.fill(0.0);
}

} // namespace vehicle_sim::telemetry
