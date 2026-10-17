#include "libs/VehicleSimCore/dynamics/PacejkaTireModel.h"
#include "libs/VehicleSimCore/terrain/HydroplaningModel.h"

#include <algorithm>
#include <cmath>

namespace vehicle_sim::dynamics {

PacejkaTireModel::PacejkaTireModel(const PacejkaConfig& config) noexcept
    : mConfig { config }
{
}

void PacejkaTireModel::loadPreset(TirePresetType preset) noexcept
{
    mConfig = getPreset(preset);
    reset();
}

[[nodiscard]] PacejkaConfig PacejkaTireModel::getPreset(TirePresetType preset) noexcept
{
    PacejkaConfig config {};
    switch (preset) {
    case TirePresetType::RacingSlick: {
        config.wheelRadius = 0.33;
        config.nominalLoadFz = 4500.0;
        config.loadSensitivity = 0.06;
        config.longitudinal = PacejkaCoeffs { 14.0, 1.70, 1.45, 0.10, 0.0, 0.0 };
        config.lateral = PacejkaCoeffs { 13.5, 1.55, 1.40, -0.10, 0.0, 0.0 };
        config.initialTrailDt = 0.025;
        config.trailStiffnessBt = 16.0;
        config.trailShapeCt = 1.5;
        config.relaxationLong = 0.12;
        config.relaxationLat = 0.15;
        config.rollingResistanceCoeff = 0.012;
        config.combinedMethod = CombinedSlipMethod::FrictionEllipse;
        break;
    }
    case TirePresetType::OffRoadKnobby: {
        config.wheelRadius = 0.40;
        config.nominalLoadFz = 4000.0;
        config.loadSensitivity = 0.12;
        config.longitudinal = PacejkaCoeffs { 6.5, 1.45, 0.85, -1.20, 0.0, 0.0 };
        config.lateral = PacejkaCoeffs { 5.5, 1.25, 0.80, -1.40, 0.0, 0.0 };
        config.initialTrailDt = 0.045;
        config.trailStiffnessBt = 8.0;
        config.trailShapeCt = 1.2;
        config.relaxationLong = 0.35;
        config.relaxationLat = 0.40;
        config.rollingResistanceCoeff = 0.035;
        config.combinedMethod = CombinedSlipMethod::FrictionEllipse;
        break;
    }
    case TirePresetType::LowGripVintage: {
        config.wheelRadius = 0.35;
        config.nominalLoadFz = 3500.0;
        config.loadSensitivity = 0.15;
        config.longitudinal = PacejkaCoeffs { 6.0, 1.40, 0.70, -0.80, 0.0, 0.0 };
        config.lateral = PacejkaCoeffs { 5.0, 1.20, 0.65, -1.00, 0.0, 0.0 };
        config.initialTrailDt = 0.050;
        config.trailStiffnessBt = 7.0;
        config.trailShapeCt = 1.1;
        config.relaxationLong = 0.30;
        config.relaxationLat = 0.35;
        config.rollingResistanceCoeff = 0.025;
        config.combinedMethod = CombinedSlipMethod::FrictionEllipse;
        break;
    }
    case TirePresetType::StandardRoad:
    default: {
        config.wheelRadius = 0.32;
        config.nominalLoadFz = 4000.0;
        config.loadSensitivity = 0.10;
        config.longitudinal = PacejkaCoeffs { 10.0, 1.65, 1.00, -0.50, 0.0, 0.0 };
        config.lateral = PacejkaCoeffs { 9.0, 1.35, 0.95, -0.70, 0.0, 0.0 };
        config.initialTrailDt = 0.035;
        config.trailStiffnessBt = 12.0;
        config.trailShapeCt = 1.4;
        config.relaxationLong = 0.20;
        config.relaxationLat = 0.25;
        config.rollingResistanceCoeff = 0.015;
        config.combinedMethod = CombinedSlipMethod::FrictionEllipse;
        break;
    }
    }
    return config;
}

[[nodiscard]] double PacejkaTireModel::evaluateMagicFormula(
    double slip, const PacejkaCoeffs& coeffs, double peakForce) noexcept
{
    if (peakForce <= 0.0) {
        return 0.0;
    }

    const double sigma1 { slip + coeffs.Sh };
    const double bSigma { coeffs.B * sigma1 };
    const double clampedE { std::clamp(coeffs.E, -2.0, 1.0) };
    const double theta { std::atan(bSigma) };
    const double inner { bSigma - (clampedE * (bSigma - theta)) };
    const double force { (peakForce * std::sin(coeffs.C * std::atan(inner))) + coeffs.Sv };

    if (std::isnan(force) || std::isinf(force)) {
        return 0.0;
    }
    return force;
}

[[nodiscard]] double PacejkaTireModel::computeSlipRatio(double vx, double omega, double radius) noexcept
{
    const double vWheel { omega * radius };
    if (std::abs(vx) < 1e-4 && std::abs(vWheel) < 1e-4) {
        return 0.0;
    }

    const double vMax { std::max({ std::abs(vx), std::abs(vWheel), 0.15 }) };
    double kappa { (vWheel - vx) / vMax };

    const double vSpeed { std::max(std::abs(vx), std::abs(vWheel)) };
    if (vSpeed < 0.15) {
        const double blend { vSpeed / 0.15 };
        kappa *= blend;
    }

    return std::clamp(kappa, -1.5, 1.5);
}

[[nodiscard]] double PacejkaTireModel::computeSlipAngle(double vx, double vy) noexcept
{
    if (std::abs(vx) < 1e-4 && std::abs(vy) < 1e-4) {
        return 0.0;
    }

    const double vRef { std::max(std::abs(vx), 0.15) };
    double alpha { -std::atan2(vy, vRef) };

    const double vAbs { std::abs(vx) };
    if (vAbs < 0.15) {
        const double blend { vAbs / 0.15 };
        alpha *= blend;
    }

    constexpr double kHalfPi { 1.5707963267948966 };
    return std::clamp(alpha, -kHalfPi, kHalfPi);
}

void PacejkaTireModel::updateRelaxation(double dt, double vx, double targetKappa, double targetAlpha) noexcept
{
    if (dt <= 0.0) {
        return;
    }

    const double vSpeed { std::abs(vx) + 0.10 };

    const double sigmaLong { std::max(mConfig.relaxationLong, 0.01) };
    const double tauLong { sigmaLong / vSpeed };
    const double decayLong { std::exp(-dt / tauLong) };
    mEffectiveKappa = targetKappa + ((mEffectiveKappa - targetKappa) * decayLong);

    const double sigmaLat { std::max(mConfig.relaxationLat, 0.01) };
    const double tauLat { sigmaLat / vSpeed };
    const double decayLat { std::exp(-dt / tauLat) };
    mEffectiveAlpha = targetAlpha + ((mEffectiveAlpha - targetAlpha) * decayLat);
}

[[nodiscard]] TireForces PacejkaTireModel::evaluateForces(const TireState& state, double dt) noexcept
{
    if (state.normalLoad <= 0.0) {
        return TireForces {};
    }

    const double geomKappa { computeSlipRatio(state.forwardSpeed, state.angularVelocity, mConfig.wheelRadius) };
    const double geomAlpha { computeSlipAngle(state.forwardSpeed, state.lateralSpeed) };

    updateRelaxation(dt, state.forwardSpeed, geomKappa, geomAlpha);

    double hydroLift { 0.0 };
    double effFz { state.normalLoad };
    double sprayDrag { 0.0 };
    double wetScale { 1.0 };
    bool isHydro { false };

    if (state.waterFilmDepth > 0.0) {
        terrain::HydroplaningConfig hConfig {};
        hConfig.tirePressurePsi = state.tirePressurePsi;
        hConfig.treadDepth = state.treadDepth;
        hConfig.voidRatio = state.treadVoidRatio;

        terrain::HydroplaningState hState {};
        hState.forwardVelocity = state.forwardSpeed;
        hState.normalLoad = state.normalLoad;
        hState.patchWidth = 2.0 * mConfig.wheelRadius * 0.35;
        hState.patchLength = 2.0 * mConfig.wheelRadius * 0.25;
        hState.waterDepth = state.waterFilmDepth;
        hState.velocityVector = math::Vec2 { state.forwardSpeed, state.lateralSpeed };

        const auto hRes = terrain::HydroplaningSolver::evaluateHydroplaning(hState, hConfig);
        hydroLift = hRes.hydroLiftForce;
        effFz = hRes.effectiveNormalLoad;
        sprayDrag = hRes.sprayDragForce;
        wetScale = hRes.frictionScale;
        isHydro = hRes.isHydroplaning;
    }

    if (effFz <= 0.0) {
        return TireForces {
            0.0, 0.0, 0.0, 0.0,
            geomKappa, geomAlpha, mEffectiveKappa, mEffectiveAlpha,
            false, hydroLift, 0.0, sprayDrag, wetScale, isHydro
        };
    }

    const double Fz { effFz };
    const double Fz0 { std::max(mConfig.nominalLoadFz, 100.0) };
    const double loadFactor { 1.0 - (mConfig.loadSensitivity * ((Fz - Fz0) / Fz0)) };
    const double muLoad { std::clamp(loadFactor, 0.40, 1.60) };
    const double surfFriction { std::max(state.surfaceFriction, 0.0) * wetScale };

    const double Dx { mConfig.longitudinal.D * surfFriction * muLoad * Fz };
    const double Dy { mConfig.lateral.D * surfFriction * muLoad * Fz };

    PacejkaCoeffs latCoeffs { mConfig.lateral };
    latCoeffs.Sv += (0.02 * state.camberAngle * Fz);

    const double Fx0 { evaluateMagicFormula(mEffectiveKappa, mConfig.longitudinal, Dx) };
    const double Fy0 { evaluateMagicFormula(mEffectiveAlpha, latCoeffs, Dy) };

    double Fx { Fx0 };
    double Fy { Fy0 };
    bool isSaturated { false };

    const double termX0 { (Dx > 1e-3) ? (Fx0 / Dx) : 0.0 };
    const double termY0 { (Dy > 1e-3) ? (Fy0 / Dy) : 0.0 };
    const double rho0 { std::hypot(termX0, termY0) };

    if (mConfig.combinedMethod == CombinedSlipMethod::FrictionEllipse) {
        if (rho0 > 1.0) {
            Fx = Fx0 / rho0;
            Fy = Fy0 / rho0;
            isSaturated = true;
        } else {
            isSaturated = (rho0 > 0.95);
        }
    } else {
        const double gXa { std::cos(1.0 * std::atan(6.0 * mEffectiveAlpha)) };
        const double gYk { std::cos(1.0 * std::atan(6.0 * mEffectiveKappa)) };
        Fx = Fx0 * gXa;
        Fy = Fy0 * gYk;
        isSaturated = (rho0 > 0.95);
    }

    const double trail { mConfig.initialTrailDt
        * std::cos(mConfig.trailShapeCt * std::atan(mConfig.trailStiffnessBt * mEffectiveAlpha)) };
    const double Mz { -trail * Fy };

    const double vx { state.forwardSpeed };
    const double speedFactor { std::clamp(vx / 0.10, -1.0, 1.0) };
    const double Frr { -mConfig.rollingResistanceCoeff * Fz * std::max(state.surfaceDrag, 0.0) * speedFactor };

    return TireForces {
        Fx, Fy, Mz, Frr,
        geomKappa, geomAlpha, mEffectiveKappa, mEffectiveAlpha,
        isSaturated, hydroLift, effFz, sprayDrag, wetScale, isHydro
    };
}

void PacejkaTireModel::reset() noexcept
{
    mEffectiveKappa = 0.0;
    mEffectiveAlpha = 0.0;
}

[[nodiscard]] const PacejkaConfig& PacejkaTireModel::getConfig() const noexcept
{
    return mConfig;
}

void PacejkaTireModel::setConfig(const PacejkaConfig& config) noexcept
{
    mConfig = config;
}

} // namespace vehicle_sim::dynamics
