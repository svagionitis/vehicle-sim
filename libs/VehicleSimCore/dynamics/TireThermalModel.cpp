#include "libs/VehicleSimCore/dynamics/TireThermalModel.h"

namespace vehicle_sim::dynamics {

TireThermalModel::TireThermalModel(const TireThermalConfig& config) noexcept
    : mConfig { config }
{
    reset(config.referenceColdTemp);
}

void TireThermalModel::reset(double ambientTemp) noexcept
{
    mState.surfaceTemp = ambientTemp;
    mState.carcassTemp = ambientTemp;
    mState.cavityTemp = ambientTemp;
    mState.inflationPressure = mConfig.baseInflationPressure;
    mState.effectiveMuMultiplier = evaluateGripFactor(ambientTemp);
    mState.frictionPower = 0.0;
    mState.hysteresisPower = 0.0;
    mState.isOverheated = false;
    mState.isSmoking = false;
}

double TireThermalModel::computeConvection(double airSpeed) const noexcept
{
    constexpr double refSpeed { 10.0 };
    const double safeSpeed { std::max(0.0, airSpeed) };
    const double speedRatio { safeSpeed / refSpeed };
    const double speedFactor { std::pow(speedRatio, 0.8) };
    return mConfig.baseConvectionCoeff * (1.0 + mConfig.convectionSpeedScale * speedFactor);
}

void TireThermalModel::updatePressure() noexcept
{
    constexpr double kelvinZero { 273.15 };
    const double refKelvin { mConfig.referenceColdTemp + kelvinZero };
    const double cavityKelvin { std::max(1.0, mState.cavityTemp + kelvinZero) };
    mState.inflationPressure = mConfig.baseInflationPressure * (cavityKelvin / refKelvin);
}

double TireThermalModel::evaluateGripFactor(double surfaceTemp) const noexcept
{
    // Cold regime: reduced grip when below optimal threshold
    const double coldDelta { (surfaceTemp - mConfig.coldTempMidpoint) / std::max(0.1, mConfig.coldSlopeWidth) };
    const double coldFactor { mConfig.coldGripDeficit / (1.0 + std::exp(coldDelta)) };

    // Hot regime: progressive degradation when exceeding optimal range
    const double overTemp { std::max(0.0, surfaceTemp - mConfig.optimalTempMax) };
    const double hotFactor { mConfig.maxThermalDecay * std::tanh(overTemp / std::max(1.0, mConfig.hotDegradationRate)) };

    const double rawGrip { 1.0 - coldFactor - hotFactor };
    const double minGrip { std::max(0.1, 1.0 - mConfig.maxThermalDecay) };
    return std::clamp(rawGrip, minGrip, 1.0);
}

void TireThermalModel::update(double tractiveForce,
                             double corneringForce,
                             double normalLoad,
                             double slipVelLong,
                             double slipVelLat,
                             double wheelAngularSpeed,
                             double wheelRadius,
                             double relativeAirSpeed,
                             double ambientTemp,
                             double roadTemp,
                             double dt) noexcept
{
    if (dt <= 0.0) {
        return;
    }

    // 1. Contact patch scrub friction power
    const double absFlong { std::abs(tractiveForce) };
    const double absFlat { std::abs(corneringForce) };
    const double absVsx { std::abs(slipVelLong) };
    const double absVsy { std::abs(slipVelLat) };
    const double frictionPower { mConfig.thermalConversionEff * (absFlong * absVsx + absFlat * absVsy) };

    // 2. Carcass cyclic flexing / rolling hysteresis power
    const double safeLoad { std::max(0.0, normalLoad) };
    const double linearSpeed { std::abs(wheelAngularSpeed * wheelRadius) };
    const double hysteresisPower { mConfig.hysteresisCoeff * safeLoad * linearSpeed };

    mState.frictionPower = frictionPower;
    mState.hysteresisPower = hysteresisPower;

    // 3. Convective heat dissipation scaling with relative airspeed
    const double hConv { computeConvection(relativeAirSpeed) };
    constexpr double areaSurf { 0.18 };    // Exposed contact/tread area in m^2
    constexpr double areaCarcass { 0.32 }; // Sidewall & bulk carcass area in m^2

    // Substep integration for numerical stability under large heat fluxes
    constexpr int kSubSteps { 4 };
    const double subDt { dt / static_cast<double>(kSubSteps) };

    for (int step { 0 }; step < kSubSteps; ++step) {
        // Surface heat fluxes
        const double qConvSurf { hConv * areaSurf * (mState.surfaceTemp - ambientTemp) };
        const double qRoad { mConfig.roadConductance * (mState.surfaceTemp - roadTemp) };
        const double qInternal { mConfig.internalConductance * (mState.surfaceTemp - mState.carcassTemp) };

        const double dSurfDt { (frictionPower - qRoad - qInternal - qConvSurf) / mConfig.surfaceThermalMass };

        // Carcass heat fluxes
        const double qConvCarcass { hConv * areaCarcass * (mState.carcassTemp - ambientTemp) };
        const double qCavity { mConfig.cavityConductance * (mState.carcassTemp - mState.cavityTemp) };

        const double dCarcassDt { (hysteresisPower + qInternal - qConvCarcass - qCavity) / mConfig.bulkThermalMass };

        // Cavity heat fluxes
        constexpr double kRimCooling { 8.0 }; // Rim conductive cooling to ambient
        const double qRim { kRimCooling * (mState.cavityTemp - ambientTemp) };
        const double dCavityDt { (qCavity - qRim) / mConfig.cavityAirThermalMass };

        // State advancement
        mState.surfaceTemp += dSurfDt * subDt;
        mState.carcassTemp += dCarcassDt * subDt;
        mState.cavityTemp += dCavityDt * subDt;

        // Physical clamping to prevent runaway numerical divergence
        mState.surfaceTemp = std::clamp(mState.surfaceTemp, -40.0, 450.0);
        mState.carcassTemp = std::clamp(mState.carcassTemp, -40.0, 350.0);
        mState.cavityTemp = std::clamp(mState.cavityTemp, -40.0, 250.0);
    }

    // 4. Update cavity inflation pressure
    updatePressure();

    // 5. Update dynamic grip multiplier
    mState.effectiveMuMultiplier = evaluateGripFactor(mState.surfaceTemp);
    mState.isOverheated = (mState.surfaceTemp > mConfig.optimalTempMax);

    // 6. Thermal smoke trigger
    const double totalSlipVel { std::sqrt(absVsx * absVsx + absVsy * absVsy) };
    mState.isSmoking = (mState.surfaceTemp >= mConfig.smokeThresholdTemp) && (totalSlipVel > 1.5);
}

double TireThermalModel::getSurfaceTemp() const noexcept
{
    return mState.surfaceTemp;
}

double TireThermalModel::getCarcassTemp() const noexcept
{
    return mState.carcassTemp;
}

double TireThermalModel::getCavityPressure() const noexcept
{
    return mState.inflationPressure;
}

double TireThermalModel::getGripMultiplier() const noexcept
{
    return mState.effectiveMuMultiplier;
}

const TireThermalState& TireThermalModel::getState() const noexcept
{
    return mState;
}

const TireThermalConfig& TireThermalModel::getConfig() const noexcept
{
    return mConfig;
}

void TireThermalModel::setConfig(const TireThermalConfig& config) noexcept
{
    mConfig = config;
    updatePressure();
    mState.effectiveMuMultiplier = evaluateGripFactor(mState.surfaceTemp);
}

} // namespace vehicle_sim::dynamics
