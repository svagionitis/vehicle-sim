#include "apps/MotionLab/TireThermalCoordinator.h"
#include "apps/MotionLab/TireSmokeParticleBuffer.h"

namespace motion_lab {

TireThermalCoordinator::TireThermalCoordinator() noexcept
{
    applyFleetConfiguration();
    reset(mAmbientAirTemp);
}

void TireThermalCoordinator::setVehicleType(MotionLabVehicleType type) noexcept
{
    mVehicleType = type;
    applyFleetConfiguration();
    reset(mAmbientAirTemp);
}

void TireThermalCoordinator::reset(double ambientTemp) noexcept
{
    mAmbientAirTemp = ambientTemp;
    for (auto& wheel : mWheels) {
        wheel.reset(ambientTemp);
    }
}

void TireThermalCoordinator::setEnvironment(double ambientAir, double trackRoad) noexcept
{
    mAmbientAirTemp = ambientAir;
    mTrackRoadTemp = trackRoad;
}

void TireThermalCoordinator::applyFleetConfiguration() noexcept
{
    vehicle_sim::dynamics::TireThermalConfig config {};

    switch (mVehicleType) {
    case MotionLabVehicleType::ClassicCar:
        mIsWheeled = true;
        config.surfaceThermalMass = 120.0;
        config.bulkThermalMass = 1200.0;
        config.optimalTempMin = 65.0;
        config.optimalTempMax = 85.0;
        config.maxThermalDecay = 0.25;
        config.baseInflationPressure = 2.0;
        config.smokeThresholdTemp = 115.0;
        break;

    case MotionLabVehicleType::RaceCar:
        mIsWheeled = true;
        config.surfaceThermalMass = 100.0;
        config.bulkThermalMass = 950.0;
        config.optimalTempMin = 85.0;
        config.optimalTempMax = 105.0;
        config.maxThermalDecay = 0.38;
        config.baseInflationPressure = 1.8;
        config.smokeThresholdTemp = 110.0;
        break;

    case MotionLabVehicleType::DriftCar:
        mIsWheeled = true;
        config.surfaceThermalMass = 110.0;
        config.bulkThermalMass = 1100.0;
        config.optimalTempMin = 75.0;
        config.optimalTempMax = 95.0;
        config.maxThermalDecay = 0.35;
        config.baseInflationPressure = 1.6;
        config.smokeThresholdTemp = 105.0;
        break;

    case MotionLabVehicleType::MonsterTruck:
        mIsWheeled = true;
        config.surfaceThermalMass = 600.0;
        config.bulkThermalMass = 6500.0;
        config.optimalTempMin = 50.0;
        config.optimalTempMax = 75.0;
        config.maxThermalDecay = 0.22;
        config.baseInflationPressure = 1.2;
        config.hysteresisCoeff = 0.022;
        config.smokeThresholdTemp = 125.0;
        break;

    case MotionLabVehicleType::SemiTruck:
        mIsWheeled = true;
        config.surfaceThermalMass = 450.0;
        config.bulkThermalMass = 5200.0;
        config.optimalTempMin = 60.0;
        config.optimalTempMax = 90.0;
        config.maxThermalDecay = 0.30;
        config.baseInflationPressure = 7.5;
        config.smokeThresholdTemp = 120.0;
        break;

    case MotionLabVehicleType::Forklift:
        mIsWheeled = true;
        config.surfaceThermalMass = 180.0;
        config.bulkThermalMass = 1600.0;
        config.optimalTempMin = 45.0;
        config.optimalTempMax = 70.0;
        config.maxThermalDecay = 0.30;
        config.baseInflationPressure = 0.0; // Solid polyurethane
        config.smokeThresholdTemp = 110.0;
        break;

    case MotionLabVehicleType::TrackedTank:
        mIsWheeled = true;
        config.surfaceThermalMass = 350.0;
        config.bulkThermalMass = 4500.0;
        config.optimalTempMin = 50.0;
        config.optimalTempMax = 80.0;
        config.maxThermalDecay = 0.20;
        config.baseInflationPressure = 0.0;
        config.smokeThresholdTemp = 120.0;
        break;

    case MotionLabVehicleType::Rocket:
    case MotionLabVehicleType::Helicopter:
    case MotionLabVehicleType::Hovercraft:
    case MotionLabVehicleType::AntiGravity:
    case MotionLabVehicleType::QuadcopterDrone:
    default:
        mIsWheeled = false;
        break;
    }

    for (auto& wheel : mWheels) {
        wheel.setConfig(config);
    }
}

void TireThermalCoordinator::updateWheel(std::size_t index,
                                        double fx,
                                        double fy,
                                        double fz,
                                        double vsx,
                                        double vsy,
                                        double omega,
                                        double radius,
                                        double airSpeed,
                                        const vehicle_sim::math::Vec2& worldPos,
                                        double dt,
                                        TireSmokeParticleBuffer& smokeBuffer) noexcept
{
    if (index >= 4 || !mIsWheeled) {
        return;
    }

    vehicle_sim::dynamics::TireThermalModel& model { mWheels[index] };
    model.update(fx, fy, fz, vsx, vsy, omega, radius, airSpeed, mAmbientAirTemp, mTrackRoadTemp, dt);

    const vehicle_sim::dynamics::TireThermalState& state { model.getState() };
    if (state.isSmoking) {
        const double intensity { std::min(1.0, (state.surfaceTemp - 105.0) / 40.0) };
        const vehicle_sim::math::Vec2 slipVec { vsx, vsy };
        smokeBuffer.emit(worldPos, slipVec, intensity);
    }
}

const vehicle_sim::dynamics::TireThermalState& TireThermalCoordinator::getWheelState(std::size_t wheelIndex) const noexcept
{
    const std::size_t safeIndex { std::min(wheelIndex, static_cast<std::size_t>(3)) };
    return mWheels[safeIndex].getState();
}

const vehicle_sim::dynamics::TireThermalModel& TireThermalCoordinator::getWheelModel(std::size_t wheelIndex) const noexcept
{
    const std::size_t safeIndex { std::min(wheelIndex, static_cast<std::size_t>(3)) };
    return mWheels[safeIndex];
}

MotionLabVehicleType TireThermalCoordinator::getVehicleType() const noexcept
{
    return mVehicleType;
}

bool TireThermalCoordinator::isWheeledVehicle() const noexcept
{
    return mIsWheeled;
}

double TireThermalCoordinator::getAmbientAirTemp() const noexcept
{
    return mAmbientAirTemp;
}

double TireThermalCoordinator::getTrackRoadTemp() const noexcept
{
    return mTrackRoadTemp;
}

} // namespace motion_lab
