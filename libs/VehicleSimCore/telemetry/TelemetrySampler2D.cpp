#include "libs/VehicleSimCore/telemetry/TelemetrySampler2D.h"

#include <algorithm>
#include <cmath>

#include "libs/VehicleSimCore/dynamics/ArcadeCarDynamics.h"
#include "libs/VehicleSimCore/dynamics/IVehicleDynamics.h"

namespace vehicle_sim::telemetry {

namespace {
    constexpr double kStandardGravity { 9.80665 };
    constexpr double kMinTimestep { 1e-6 };
} // namespace

TelemetrySampler2D::TelemetrySampler2D(double filterAlpha) noexcept
    : m_filterAlpha { std::clamp(filterAlpha, 0.01, 1.0) }
{
}

void TelemetrySampler2D::reset() noexcept
{
    m_tick = 0;
    m_prevVelocity = math::Vec2 {};
    m_filteredAcceleration = math::Vec2 {};
    m_initialized = false;
}

void TelemetrySampler2D::setFilterAlpha(double alpha) noexcept
{
    m_filterAlpha = std::clamp(alpha, 0.01, 1.0);
}

double TelemetrySampler2D::getFilterAlpha() const noexcept
{
    return m_filterAlpha;
}

TelemetryFrame2D TelemetrySampler2D::sample(
    const physics::RigidBody2D& body, const dynamics::IVehicleDynamics* dynamics, double timestamp, double dt) noexcept
{
    TelemetryFrame2D frame {};
    frame.tickNumber = m_tick++;
    frame.timestamp = timestamp;

    frame.position = body.getPosition();
    frame.velocity = body.getLinearVelocity();
    frame.heading = body.getOrientation();
    frame.angularVelocity = body.getAngularVelocity();

    const double cosH { std::cos(frame.heading) };
    const double sinH { std::sin(frame.heading) };
    frame.forwardSpeed = (frame.velocity.x * cosH) + (frame.velocity.y * sinH);

    if (m_initialized && (dt > kMinTimestep)) {
        const math::Vec2 rawAccel { (frame.velocity - m_prevVelocity) * (1.0 / dt) };
        m_filteredAcceleration = (m_filteredAcceleration * (1.0 - m_filterAlpha)) + (rawAccel * m_filterAlpha);
    } else {
        m_filteredAcceleration = math::Vec2 { 0.0, 0.0 };
        m_initialized = true;
    }
    m_prevVelocity = frame.velocity;

    frame.acceleration = m_filteredAcceleration;
    const double longAccel { (m_filteredAcceleration.x * cosH) + (m_filteredAcceleration.y * sinH) };
    const double latAccel { (-m_filteredAcceleration.x * sinH) + (m_filteredAcceleration.y * cosH) };
    frame.longitudinalG = longAccel / kStandardGravity;
    frame.lateralG = latAccel / kStandardGravity;

    if (dynamics != nullptr) {
        frame.isAirborne = dynamics->isAirborne();
    }

    return frame;
}

TelemetryFrame2D TelemetrySampler2D::sampleArcadeCar(
    const physics::RigidBody2D& body, const dynamics::ArcadeCarDynamics& car, double timestamp, double dt) noexcept
{
    TelemetryFrame2D frame { sample(body, &car, timestamp, dt) };

    const auto& input { car.getInput() };
    frame.throttleInput = input.throttle;
    frame.steeringInput = input.steer;
    frame.brakeInput = input.brake;
    frame.handbrakeInput = (input.handbrake == dynamics::HandbrakeState::Engaged) ? 1.0 : 0.0;

    frame.wheelCount = dynamics::ArcadeCarDynamics::kWheelCount;
    for (std::size_t i { 0 }; i < dynamics::ArcadeCarDynamics::kWheelCount; ++i) {
        const auto& wheel { car.getWheelTelemetry(i) };
        frame.wheels[i].position = wheel.worldPosition;
        frame.wheels[i].contactForce = wheel.force;
        frame.wheels[i].slipAngle = wheel.steerAngle;
        frame.wheels[i].surfaceFriction = wheel.surfaceGrip;
        frame.wheels[i].normalForce = wheel.force.length();
        frame.wheels[i].isGrounded = !car.isAirborne();
    }

    frame.altitude = car.getFlightHeight();
    frame.auxChannels[0] = car.getNitroFuel();

    return frame;
}

} // namespace vehicle_sim::telemetry
