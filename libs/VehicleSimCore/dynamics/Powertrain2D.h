#pragma once

#include <cstdint>
#include <memory>

#include "libs/VehicleSimCore/dynamics/Differential2D.h"
#include "libs/VehicleSimCore/dynamics/FlywheelClutch2D.h"
#include "libs/VehicleSimCore/dynamics/IEngine2D.h"
#include "libs/VehicleSimCore/dynamics/Transmission2D.h"

namespace vehicle_sim::dynamics {

/// @enum DrivetrainLayout
/// @brief Driven wheel arrangement topology.
enum class DrivetrainLayout : std::uint8_t {
    FrontWheelDrive = 0U, ///< Engine drives front wheels only.
    RearWheelDrive = 1U,  ///< Engine drives rear wheels only.
    AllWheelDrive = 2U    ///< Engine drives all four wheels via center split.
};

/// @struct PowertrainConfig
/// @brief Master configuration struct assembling all powertrain subsystems.
struct PowertrainConfig {
    DrivetrainLayout layout { DrivetrainLayout::RearWheelDrive };
    double centerSplitFront { 0.40 }; ///< Front torque split fraction for AWD in [0..1].
    double wheelRadius { 0.33 }; ///< Nominal tire rolling radius in meters.
};

/// @struct PowertrainState
/// @brief Complete snapshot of dynamic powertrain variables for telemetry/audio.
struct PowertrainState {
    double engineRpm { 0.0 }; ///< Crankshaft / rotor speed in RPM.
    double engineTorque { 0.0 }; ///< Prime mover gross output in N*m.
    double clutchTorque { 0.0 }; ///< Torque transmitted through clutch in N*m.
    double propTorque { 0.0 }; ///< Propeller shaft torque in N*m.
    double leftWheelTorque { 0.0 }; ///< Left drive wheel torque in N*m.
    double rightWheelTorque { 0.0 }; ///< Right drive wheel torque in N*m.
    double leftWheelSpeed { 0.0 }; ///< Left wheel speed in rad/s.
    double rightWheelSpeed { 0.0 }; ///< Right wheel speed in rad/s.
    std::int32_t activeGear { 1 }; ///< Engaged transmission gear.
    bool isShifting { false }; ///< True during gear changes.
    bool isRedline { false }; ///< True if engine is rev-limiting.
    bool isStalled { false }; ///< True if ICE has stalled.
    bool isClutchLocked { false }; ///< True if clutch plates are statically locked.
};

/// @class Powertrain2D
/// @brief Unified modular vehicle powertrain assembling engine, clutch, transmission, and differential.
class Powertrain2D {
public:
    /// @brief Constructs default powertrain with internal combustion engine and Salisbury LSD.
    Powertrain2D() noexcept;

    /// @brief Parameterized constructor accepting modular subsystem instances.
    /// @param[in] engine Unique pointer to prime mover implementation.
    /// @param[in] clutchConfig Flywheel and clutch friction settings.
    /// @param[in] transConfig Transmission ratios and shift parameters.
    /// @param[in] diffConfig Axle differential parameters.
    /// @param[in] powertrainConfig Drivetrain layout and geometry.
    Powertrain2D(std::unique_ptr<IEngine2D> engine,
                 const ClutchConfig& clutchConfig,
                 const TransmissionConfig& transConfig,
                 const DifferentialConfig& diffConfig,
                 const PowertrainConfig& powertrainConfig) noexcept;

    /// @brief Destructor.
    ~Powertrain2D() noexcept = default;

    /// @brief Advances complete powertrain simulation step.
    /// @param[in] throttle Driver throttle input in [0.0, 1.0].
    /// @param[in] brake Driver service brake input in [0.0, 1.0].
    /// @param[in] clutchPedal Driver clutch pedal in [0.0, 1.0] (0 = engaged).
    /// @param[in] leftWheelSpeed Left drive wheel angular velocity in rad/s.
    /// @param[in] rightWheelSpeed Right drive wheel angular velocity in rad/s.
    /// @param[in] dt Timestep duration in seconds.
    /// @return Evaluated powertrain state snapshot.
    [[nodiscard]] PowertrainState step(double throttle,
                                       double brake,
                                       double clutchPedal,
                                       double leftWheelSpeed,
                                       double rightWheelSpeed,
                                       double dt) noexcept;

    /// @brief Resets all powertrain subsystems.
    void reset() noexcept;

    /// @brief Requests a sequential upshift.
    /// @return True if upshift initiated.
    bool shiftUp() noexcept;

    /// @brief Requests a sequential downshift.
    /// @return True if downshift initiated.
    bool shiftDown() noexcept;

    /// @brief Selects a specific transmission gear directly.
    /// @param[in] gear Target gear index (-1 = Rev, 0 = N, 1..N = Fwd).
    /// @return True if shift accepted.
    bool selectGear(std::int32_t gear) noexcept;

    /// @brief Swaps active prime mover dynamically (e.g., ICE to EV).
    /// @param[in] newEngine Unique pointer to new IEngine2D implementation.
    void setEngine(std::unique_ptr<IEngine2D> newEngine) noexcept;

    /// @brief Accesses underlying prime mover.
    /// @return Reference to active IEngine2D.
    [[nodiscard]] IEngine2D& getEngine() noexcept;

    /// @brief Accesses underlying transmission.
    /// @return Reference to Transmission2D.
    [[nodiscard]] Transmission2D& getTransmission() noexcept;

    /// @brief Accesses underlying differential.
    /// @return Reference to Differential2D.
    [[nodiscard]] Differential2D& getDifferential() noexcept;

    /// @brief Queries current powertrain state snapshot.
    /// @return Cached PowertrainState.
    [[nodiscard]] const PowertrainState& getState() const noexcept;

    /// @brief Queries drivetrain layout.
    /// @return Current DrivetrainLayout.
    [[nodiscard]] DrivetrainLayout getLayout() const noexcept;

    /// @brief Updates drivetrain layout.
    /// @param[in] layout New DrivetrainLayout.
    void setLayout(DrivetrainLayout layout) noexcept;

private:
    std::unique_ptr<IEngine2D> engine_;
    FlywheelClutch2D clutch_;
    Transmission2D transmission_;
    Differential2D differential_;
    PowertrainConfig config_ {};
    PowertrainState lastState_ {};
};

} // namespace vehicle_sim::dynamics
