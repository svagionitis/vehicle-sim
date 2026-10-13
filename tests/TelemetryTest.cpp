#include <gtest/gtest.h>

#include <cmath>
#include <sstream>
#include <vector>

#include "libs/VehicleSimCore/dynamics/ArcadeCarDynamics.h"
#include "libs/VehicleSimCore/physics/RigidBody2D.h"
#include "libs/VehicleSimCore/telemetry/CsvTelemetryExporter.h"
#include "libs/VehicleSimCore/telemetry/KlvTelemetryExporter.h"
#include "libs/VehicleSimCore/telemetry/TelemetryFrame2D.h"
#include "libs/VehicleSimCore/telemetry/TelemetryRingBuffer2D.h"
#include "libs/VehicleSimCore/telemetry/TelemetrySampler2D.h"

namespace vehicle_sim::telemetry::testing {

TEST(TelemetryFrame2DTest, DefaultInitializationAndReset)
{
    TelemetryFrame2D frame {};
    EXPECT_EQ(frame.tickNumber, 0U);
    EXPECT_DOUBLE_EQ(frame.timestamp, 0.0);
    EXPECT_DOUBLE_EQ(frame.forwardSpeed, 0.0);
    EXPECT_DOUBLE_EQ(frame.longitudinalG, 0.0);
    EXPECT_DOUBLE_EQ(frame.lateralG, 0.0);
    EXPECT_FALSE(frame.isAirborne);
    EXPECT_EQ(frame.wheelCount, 0U);

    frame.tickNumber = 42U;
    frame.timestamp = 1.25;
    frame.forwardSpeed = 25.0;
    frame.isAirborne = true;
    frame.reset();

    EXPECT_EQ(frame.tickNumber, 0U);
    EXPECT_DOUBLE_EQ(frame.timestamp, 0.0);
    EXPECT_DOUBLE_EQ(frame.forwardSpeed, 0.0);
    EXPECT_FALSE(frame.isAirborne);
}

TEST(TelemetryRingBuffer2DTest, BasicPushAndQuery)
{
    TelemetryRingBuffer2D buffer { 4 };
    EXPECT_EQ(buffer.getCapacity(), 4U);
    EXPECT_EQ(buffer.getCount(), 0U);
    EXPECT_TRUE(buffer.isEmpty());
    EXPECT_FALSE(buffer.isFull());

    // Empty queries return empty frame safely
    EXPECT_DOUBLE_EQ(buffer.getLatest().timestamp, 0.0);
    EXPECT_DOUBLE_EQ(buffer.getOldest().timestamp, 0.0);
    EXPECT_DOUBLE_EQ(buffer.getFrame(0).timestamp, 0.0);
    EXPECT_DOUBLE_EQ(buffer.getPastFrame(0).timestamp, 0.0);

    TelemetryFrame2D f1 {};
    f1.tickNumber = 1;
    f1.timestamp = 0.1;
    f1.forwardSpeed = 10.0;
    f1.longitudinalG = 0.5;
    buffer.push(f1);

    EXPECT_EQ(buffer.getCount(), 1U);
    EXPECT_FALSE(buffer.isEmpty());
    EXPECT_FALSE(buffer.isFull());
    EXPECT_DOUBLE_EQ(buffer.getLatest().timestamp, 0.1);
    EXPECT_DOUBLE_EQ(buffer.getOldest().timestamp, 0.1);

    TelemetryFrame2D f2 {};
    f2.tickNumber = 2;
    f2.timestamp = 0.2;
    f2.forwardSpeed = 15.0;
    f2.longitudinalG = -1.2;
    buffer.push(f2);

    EXPECT_EQ(buffer.getCount(), 2U);
    EXPECT_DOUBLE_EQ(buffer.getLatest().timestamp, 0.2);
    EXPECT_DOUBLE_EQ(buffer.getOldest().timestamp, 0.1);
    EXPECT_DOUBLE_EQ(buffer.getFrame(0).timestamp, 0.1);
    EXPECT_DOUBLE_EQ(buffer.getFrame(1).timestamp, 0.2);
    EXPECT_DOUBLE_EQ(buffer.getPastFrame(0).timestamp, 0.2);
    EXPECT_DOUBLE_EQ(buffer.getPastFrame(1).timestamp, 0.1);
}

TEST(TelemetryRingBuffer2DTest, OverwriteAndPeakGForces)
{
    TelemetryRingBuffer2D buffer { 3 };

    for (std::size_t i { 1 }; i <= 5; ++i) {
        TelemetryFrame2D f {};
        f.tickNumber = i;
        f.timestamp = static_cast<double>(i) * 0.1;
        f.longitudinalG = (i % 2 == 0) ? -0.8 : 0.4;
        f.lateralG = static_cast<double>(i) * 0.3;
        buffer.push(f);
    }

    EXPECT_TRUE(buffer.isFull());
    EXPECT_EQ(buffer.getCount(), 3U);
    // Capacity 3: frames 3, 4, 5 retained
    EXPECT_DOUBLE_EQ(buffer.getOldest().timestamp, 0.3);
    EXPECT_DOUBLE_EQ(buffer.getLatest().timestamp, 0.5);
    EXPECT_DOUBLE_EQ(buffer.getFrame(0).timestamp, 0.3);
    EXPECT_DOUBLE_EQ(buffer.getFrame(1).timestamp, 0.4);
    EXPECT_DOUBLE_EQ(buffer.getFrame(2).timestamp, 0.5);

    double maxLongG { 0.0 };
    double maxLatG { 0.0 };
    buffer.getPeakGForces(maxLongG, maxLatG);
    EXPECT_NEAR(maxLongG, 0.8, 1e-5);
    EXPECT_NEAR(maxLatG, 1.5, 1e-5); // frame 5 has lateralG 5 * 0.3 = 1.5

    buffer.clear();
    EXPECT_TRUE(buffer.isEmpty());
    EXPECT_EQ(buffer.getCount(), 0U);
}

TEST(TelemetrySampler2DTest, KinematicSamplingAndGForces)
{
    physics::RigidBody2D body {};
    body.setPosition(math::Vec2 { 10.0, 20.0 });
    body.setOrientation(0.0);
    body.setLinearVelocity(math::Vec2 { 0.0, 0.0 });

    TelemetrySampler2D sampler { 1.0 }; // Alpha = 1.0 (unfiltered pass-through for exact test)

    // Step 0: standstill
    TelemetryFrame2D frame0 { sampler.sample(body, nullptr, 0.0, 0.01) };
    EXPECT_EQ(frame0.tickNumber, 0U);
    EXPECT_DOUBLE_EQ(frame0.position.x, 10.0);
    EXPECT_DOUBLE_EQ(frame0.position.y, 20.0);
    EXPECT_DOUBLE_EQ(frame0.forwardSpeed, 0.0);
    EXPECT_DOUBLE_EQ(frame0.longitudinalG, 0.0);
    EXPECT_DOUBLE_EQ(frame0.lateralG, 0.0);

    // Step 1: accelerate along +X (longitudinal) by 9.80665 m/s in 1.0 s => 1.0 G
    body.setLinearVelocity(math::Vec2 { 9.80665, 0.0 });
    TelemetryFrame2D frame1 { sampler.sample(body, nullptr, 1.0, 1.0) };
    EXPECT_EQ(frame1.tickNumber, 1U);
    EXPECT_DOUBLE_EQ(frame1.forwardSpeed, 9.80665);
    EXPECT_NEAR(frame1.longitudinalG, 1.0, 1e-4);
    EXPECT_NEAR(frame1.lateralG, 0.0, 1e-4);

    // Step 2: rotate 90 deg (pi/2 rad) and accelerate laterally
    body.setOrientation(1.5707963267948966); // pointing +Y
    body.setLinearVelocity(math::Vec2 { 9.80665, 9.80665 }); // Delta vel along +Y = 9.80665
    TelemetryFrame2D frame2 { sampler.sample(body, nullptr, 2.0, 1.0) };
    EXPECT_NEAR(frame2.forwardSpeed, 9.80665, 1e-4);
    // Heading +Y, so +Y acceleration is longitudinal
    EXPECT_NEAR(frame2.longitudinalG, 1.0, 1e-4);
}

TEST(TelemetrySampler2DTest, ArcadeCarSampling)
{
    physics::RigidBody2D body {};
    dynamics::ArcadeCarDynamics car {};

    dynamics::ArcadeCarInput input {};
    input.throttle = 0.85;
    input.steer = -0.4;
    input.brake = 0.1;
    input.handbrake = dynamics::HandbrakeState::Engaged;
    car.setInput(input);

    TelemetrySampler2D sampler {};
    TelemetryFrame2D frame { sampler.sampleArcadeCar(body, car, 0.05, 0.01) };

    EXPECT_NEAR(frame.throttleInput, 0.85, 1e-5);
    EXPECT_NEAR(frame.steeringInput, -0.4, 1e-5);
    EXPECT_NEAR(frame.brakeInput, 0.1, 1e-5);
    EXPECT_DOUBLE_EQ(frame.handbrakeInput, 1.0);
    EXPECT_EQ(frame.wheelCount, 4U);
}

TEST(CsvTelemetryExporterTest, HeaderAndRowFormatting)
{
    CsvTelemetryExporter exporter {};
    std::string header { exporter.getHeader() };
    EXPECT_FALSE(header.empty());
    EXPECT_NE(header.find("Time"), std::string::npos);
    EXPECT_NE(header.find("Speed"), std::string::npos);
    EXPECT_NE(header.find("LongG"), std::string::npos);

    TelemetryFrame2D frame {};
    frame.timestamp = 3.5;
    frame.tickNumber = 100;
    frame.position = math::Vec2 { 12.34, 56.78 };
    frame.forwardSpeed = 22.5;

    std::string row { exporter.formatRow(frame) };
    EXPECT_NE(row.find("3.5000"), std::string::npos);
    EXPECT_NE(row.find("100"), std::string::npos);
    EXPECT_NE(row.find("12.3400"), std::string::npos);
    EXPECT_NE(row.find("22.5000"), std::string::npos);

    TelemetryRingBuffer2D buffer { 2 };
    buffer.push(frame);

    std::ostringstream ss {};
    EXPECT_TRUE(exporter.writeCsv(buffer, ss));
    std::string outStr { ss.str() };
    EXPECT_NE(outStr.find(header), std::string::npos);
    EXPECT_NE(outStr.find("3.5000"), std::string::npos);
}

TEST(KlvTelemetryExporterTest, EncodePacketAndCrcVerification)
{
    KlvGeoReference geoRef {};
    geoRef.originLatitudeDeg = 37.7749;
    geoRef.originLongitudeDeg = -122.4194;
    geoRef.originAltitudeMeters = 50.0;

    KlvTelemetryExporter exporter { geoRef };

    TelemetryFrame2D frame {};
    frame.timestamp = 10.0;
    frame.position = math::Vec2 { 0.0, 0.0 };
    frame.heading = 0.0;
    frame.forwardSpeed = 30.0;
    frame.altitude = 150.0;

    std::vector<std::uint8_t> packet {};
    EXPECT_TRUE(exporter.encodePacket(frame, packet));
    EXPECT_GT(packet.size(), 20U);

    // Verify SMPTE 336M 16-byte key
    const std::vector<std::uint8_t> expectedKey { 0x06, 0x0E, 0x2B, 0x34, 0x02, 0x0B, 0x01, 0x01, 0x0E, 0x01, 0x03,
        0x01, 0x01, 0x00, 0x00, 0x00 };
    for (std::size_t i { 0 }; i < 16; ++i) {
        EXPECT_EQ(packet[i], expectedKey[i]);
    }

    // Verify CRC-16 checksum at the end
    const std::size_t crcLen { packet.size() - 2U };
    const std::uint16_t expectedCrc { KlvTelemetryExporter::computeCrc(packet.data(), crcLen) };
    const auto highByte { static_cast<std::uint8_t>((expectedCrc >> 8U) & 0xFFU) };
    const auto lowByte { static_cast<std::uint8_t>(expectedCrc & 0xFFU) };

    EXPECT_EQ(packet[packet.size() - 2U], highByte);
    EXPECT_EQ(packet[packet.size() - 1U], lowByte);
}

} // namespace vehicle_sim::telemetry::testing
