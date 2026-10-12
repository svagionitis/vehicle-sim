#include "libs/VehicleSimCore/math/KalmanFilter1D.h"

namespace vehicle_sim::math {

KalmanFilter1D::KalmanFilter1D(
    double initPos, double initVel, double procNoiseAcc, double measNoisePos) noexcept
    : mStatePos { initPos }
    , mStateVel { initVel }
    , mProcNoiseAcc { (procNoiseAcc > 0.0) ? procNoiseAcc : 0.0 }
    , mMeasNoisePos { (measNoisePos > 0.0) ? measNoisePos : 1.0 }
    , mInitialized { true }
{
}

void KalmanFilter1D::predict(double dt, double controlAcceleration) noexcept
{
    if (dt <= 0.0) {
        return;
    }

    // 1. State prediction: x^- = F * x + B * u
    mStatePos += (mStateVel * dt) + (0.5 * controlAcceleration * dt * dt);
    mStateVel += controlAcceleration * dt;

    // 2. Covariance prediction: P^- = F * P * F^T + Q
    const double dt2 { dt * dt };
    const double dt3 { dt2 * dt };
    const double dt4 { dt2 * dt2 };

    const double q00 { 0.25 * mProcNoiseAcc * dt4 };
    const double q01 { 0.5 * mProcNoiseAcc * dt3 };
    const double q11 { mProcNoiseAcc * dt2 };

    const double p00 { mCovariance.m00 };
    const double p01 { mCovariance.m01 };
    const double p11 { mCovariance.m11 };

    const double predP00 { p00 + (2.0 * dt * p01) + (dt2 * p11) + q00 };
    const double predP01 { p01 + (dt * p11) + q01 };
    const double predP11 { p11 + q11 };

    mCovariance.m00 = std::max(predP00, 0.0);
    mCovariance.m01 = predP01;
    mCovariance.m10 = predP01;
    mCovariance.m11 = std::max(predP11, 0.0);
}

void KalmanFilter1D::updatePosition(double measuredPos, double measNoise) noexcept
{
    const double r { (measNoise > 0.0) ? measNoise : mMeasNoisePos };
    const double innov { measuredPos - mStatePos };
    const double s { mCovariance.m00 + r };

    if (s <= 1e-12) {
        return;
    }

    // Kalman gain: K = P^- * H^T / S
    const double k0 { mCovariance.m00 / s };
    const double k1 { mCovariance.m01 / s };

    // State correction
    mStatePos += k0 * innov;
    mStateVel += k1 * innov;

    // Covariance correction: P = (I - K * H) * P^-
    const double newP00 { (1.0 - k0) * mCovariance.m00 };
    const double newP01 { (1.0 - k0) * mCovariance.m01 };
    const double newP11 { mCovariance.m11 - (k1 * mCovariance.m01) };

    mCovariance.m00 = std::max(newP00, 0.0);
    mCovariance.m01 = newP01;
    mCovariance.m10 = newP01;
    mCovariance.m11 = std::max(newP11, 0.0);
    mInitialized = true;
}

void KalmanFilter1D::updateVelocity(double measuredVel, double measNoise) noexcept
{
    const double r { (measNoise > 0.0) ? measNoise : 1.0 };
    const double innov { measuredVel - mStateVel };
    const double s { mCovariance.m11 + r };

    if (s <= 1e-12) {
        return;
    }

    // Kalman gain: K = P^- * H^T / S
    const double k0 { mCovariance.m01 / s };
    const double k1 { mCovariance.m11 / s };

    // State correction
    mStatePos += k0 * innov;
    mStateVel += k1 * innov;

    // Covariance correction: P = (I - K * H) * P^-
    const double newP00 { mCovariance.m00 - (k0 * mCovariance.m01) };
    const double newP01 { (1.0 - k1) * mCovariance.m01 };
    const double newP11 { (1.0 - k1) * mCovariance.m11 };

    mCovariance.m00 = std::max(newP00, 0.0);
    mCovariance.m01 = newP01;
    mCovariance.m10 = newP01;
    mCovariance.m11 = std::max(newP11, 0.0);
    mInitialized = true;
}

double KalmanFilter1D::update(double measuredPos, double dt, double controlAcc) noexcept
{
    if (!mInitialized) {
        mStatePos = measuredPos;
        mStateVel = 0.0;
        mInitialized = true;
        return mStatePos;
    }

    predict(dt, controlAcc);
    updatePosition(measuredPos);
    return mStatePos;
}

void KalmanFilter1D::reset(
    double initPos, double initVel, double posVariance, double velVariance) noexcept
{
    mStatePos = initPos;
    mStateVel = initVel;
    mCovariance.m00 = (posVariance >= 0.0) ? posVariance : 1.0;
    mCovariance.m01 = 0.0;
    mCovariance.m10 = 0.0;
    mCovariance.m11 = (velVariance >= 0.0) ? velVariance : 1.0;
    mInitialized = false;
}

void KalmanFilter1D::setProcessNoise(double procNoiseAcc) noexcept
{
    mProcNoiseAcc = (procNoiseAcc > 0.0) ? procNoiseAcc : 0.0;
}

void KalmanFilter1D::setMeasurementNoise(double measNoisePos) noexcept
{
    mMeasNoisePos = (measNoisePos > 0.0) ? measNoisePos : 1.0;
}

} // namespace vehicle_sim::math
