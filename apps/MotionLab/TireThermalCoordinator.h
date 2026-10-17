#pragma once

#include <array>
#include <cstddef>

#include "apps/MotionLab/MotionLabVehicleType.h"
#include "libs/VehicleSimCore/dynamics/TireThermalModel.h"
#include "libs/VehicleSimCore/math/Vec2.h"

namespace motion_lab {

class TireSmokeParticleBuffer;

/// @class TireThermalCoordinator
/// @brief Coordinates 4-wheel thermodynamic state evaluation across Motion Lab fleet.
/// @details Configures vehicle-specific thermal parameters, manages ambient
/// temperatures, updates wheel ODE states, and emits thermal smoke puffs.
class TireThermalCoordinator {
public:
    /// @brief Default constructor initializing 4 standard thermal wheels.
    TireThermalCoordinator() noexcept;

    /// @brief Configures thermodynamic parameters for active vehicle model.
    /// @param[in] type Selected Motion Lab vehicle classification.
    void setVehicleType(MotionLabVehicleType type) noexcept;

    /// @brief Resets all wheel temperatures to ambient baseline.
    /// @param[in] ambientTemp Ambient temperature in Celsius.
    void reset(double ambientTemp = 25.0) noexcept;

    /// @brief Sets ambient environmental temperatures.
    /// @param[in] ambientAir Ambient air temperature in Celsius.
    /// @param[in] trackRoad Asphalt road temperature in Celsius.
    void setEnvironment(double ambientAir, double trackRoad) noexcept;

    /// @brief Updates a single wheel thermodynamic state.
    /// @param[in] index Wheel index (0: FL, 1: FR, 2: RL, 3: RR).
    /// @param[in] fx Longitudinal tractive/braking force in Newtons.
    /// @param[in] fy Lateral cornering force in Newtons.
    /// @param[in] fz Vertical normal load in Newtons.
    /// @param[in] vsx Longitudinal sliding scrub velocity in m/s.
    /// @param[in] vsy Lateral sliding scrub velocity in m/s.
    /// @param[in] omega Wheel angular velocity in rad/s.
    /// @param[in] radius Wheel rolling radius in meters.
    /// @param[in] airSpeed Relative airspeed in m/s.
    /// @param[in] worldPos Wheel contact center world coordinates.
    /// @param[in] dt Physics timestep duration in seconds.
    /// @param[in,out] smokeBuffer Particle buffer receiving smoke puffs.
    void updateWheel(std::size_t index,
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
                     TireSmokeParticleBuffer& smokeBuffer) noexcept;

    /// @brief Queries thermal state for a specific wheel index.
    /// @param[in] wheelIndex Wheel identifier (0: FL, 1: FR, 2: RL, 3: RR).
    /// @return Const reference to TireThermalState.
    [[nodiscard]] const vehicle_sim::dynamics::TireThermalState& getWheelState(std::size_t wheelIndex) const noexcept;

    /// @brief Queries thermal model instance for a specific wheel index.
    /// @param[in] wheelIndex Wheel identifier (0: FL, 1: FR, 2: RL, 3: RR).
    /// @return Const reference to TireThermalModel.
    [[nodiscard]] const vehicle_sim::dynamics::TireThermalModel& getWheelModel(std::size_t wheelIndex) const noexcept;

    /// @brief Queries active vehicle classification.
    /// @return Active MotionLabVehicleType.
    [[nodiscard]] MotionLabVehicleType getVehicleType() const noexcept;

    /// @brief Checks if vehicle utilizes wheeled thermal modeling.
    /// @return True for wheeled vehicles, false for bypass vehicles.
    [[nodiscard]] bool isWheeledVehicle() const noexcept;

    /// @brief Queries ambient air temperature in Celsius.
    /// @return Ambient temperature in Celsius.
    [[nodiscard]] double getAmbientAirTemp() const noexcept;

    /// @brief Queries track road temperature in Celsius.
    /// @return Road surface temperature in Celsius.
    [[nodiscard]] double getTrackRoadTemp() const noexcept;

private:
    std::array<vehicle_sim::dynamics::TireThermalModel, 4> mWheels {};
    MotionLabVehicleType mVehicleType { MotionLabVehicleType::ClassicCar };
    double mAmbientAirTemp { 25.0 };
    double mTrackRoadTemp { 30.0 };
    bool mIsWheeled { true };

    void applyFleetConfiguration() noexcept;
};

} // namespace motion_lab
