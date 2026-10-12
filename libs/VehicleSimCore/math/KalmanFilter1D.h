#pragma once

#include <algorithm>
#include <cmath>

#include "libs/VehicleSimCore/math/Mat2.h"

namespace vehicle_sim::math {

/// @class KalmanFilter1D
/// @brief 1D discrete linear kinematic Kalman filter estimating position and velocity.
/// @details Implements a 2-state discrete linear Kalman filter tracking state vector
/// x = [position, velocity]^T with continuous-discrete kinematic prediction and minimum-variance
/// Bayesian sensor updates. Completely self-contained in pure C++17 with zero dynamic allocations.
class KalmanFilter1D {
public:
    /// @brief Default constructor initializing at origin with unit variance.
    constexpr KalmanFilter1D() noexcept = default;

    /// @brief Constructs a Kalman filter with specified initial state and noise parameters.
    /// @param[in] initPos Initial estimated position.
    /// @param[in] initVel Initial estimated velocity.
    /// @param[in] procNoiseAcc Process noise spectral density (acceleration variance in (units/s^2)^2).
    /// @param[in] measNoisePos Position measurement noise variance (units^2).
    KalmanFilter1D(double initPos, double initVel, double procNoiseAcc = 0.1,
        double measNoisePos = 1.0) noexcept;

    /// @brief Advances state prediction forward in time by dt.
    /// @details Computes x^- = F * x + B * u and P^- = F * P * F^T + Q.
    /// @param[in] dt Elapsed sample duration in seconds.
    /// @param[in] controlAcceleration Known control acceleration input in units/s^2 (default 0.0).
    void predict(double dt, double controlAcceleration = 0.0) noexcept;

    /// @brief Updates state estimate using a noisy position observation.
    /// @details Computes Kalman gain K and corrects state x and error covariance P.
    /// @param[in] measuredPos Observed position measurement.
    /// @param[in] measNoise Override measurement variance (if <= 0.0, uses configured default).
    void updatePosition(double measuredPos, double measNoise = 0.0) noexcept;

    /// @brief Updates state estimate using a noisy velocity observation.
    /// @param[in] measuredVel Observed velocity measurement.
    /// @param[in] measNoise Override measurement variance (must be > 0.0).
    void updateVelocity(double measuredVel, double measNoise = 1.0) noexcept;

    /// @brief Convenience update performing both predict(dt, acc) and updatePosition(measuredPos).
    /// @details If uninitialized, snaps state directly to measuredPos with zero velocity.
    /// @param[in] measuredPos Observed position measurement.
    /// @param[in] dt Elapsed sample duration in seconds.
    /// @param[in] controlAcc Known control acceleration input in units/s^2 (default 0.0).
    /// @return Estimated position after measurement update.
    [[nodiscard]] double update(double measuredPos, double dt, double controlAcc = 0.0) noexcept;

    /// @brief Resets filter state to specified position and velocity.
    /// @param[in] initPos Position to initialize.
    /// @param[in] initVel Velocity to initialize.
    /// @param[in] posVariance Initial position variance (default 1.0).
    /// @param[in] velVariance Initial velocity variance (default 1.0).
    void reset(double initPos = 0.0, double initVel = 0.0, double posVariance = 1.0,
        double velVariance = 1.0) noexcept;

    /// @brief Sets process noise acceleration spectral density.
    /// @param[in] procNoiseAcc Acceleration noise variance.
    void setProcessNoise(double procNoiseAcc) noexcept;

    /// @brief Sets default position measurement noise variance.
    /// @param[in] measNoisePos Position variance.
    void setMeasurementNoise(double measNoisePos) noexcept;

    /// @brief Returns the estimated position.
    /// @return Current state position.
    [[nodiscard]] constexpr double position() const noexcept
    {
        return mStatePos;
    }

    /// @brief Returns the estimated velocity.
    /// @return Current state velocity.
    [[nodiscard]] constexpr double velocity() const noexcept
    {
        return mStateVel;
    }

    /// @brief Returns position estimation error variance P_00.
    /// @return Position error variance.
    [[nodiscard]] constexpr double positionVariance() const noexcept
    {
        return mCovariance.m00;
    }

    /// @brief Returns velocity estimation error variance P_11.
    /// @return Velocity error variance.
    [[nodiscard]] constexpr double velocityVariance() const noexcept
    {
        return mCovariance.m11;
    }

    /// @brief Returns full 2x2 estimation error covariance matrix P.
    /// @return Covariance matrix Mat2.
    [[nodiscard]] constexpr const Mat2& covariance() const noexcept
    {
        return mCovariance;
    }

    /// @brief Checks whether the filter has received at least one sample.
    /// @return True if initialized.
    [[nodiscard]] constexpr bool isInitialized() const noexcept
    {
        return mInitialized;
    }

private:
    double mStatePos { 0.0 };
    double mStateVel { 0.0 };
    Mat2 mCovariance { 1.0, 0.0, 0.0, 1.0 };

    double mProcNoiseAcc { 0.1 };
    double mMeasNoisePos { 1.0 };
    bool mInitialized { false };
};

} // namespace vehicle_sim::math
