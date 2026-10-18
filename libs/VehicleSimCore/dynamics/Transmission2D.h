#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

namespace vehicle_sim::dynamics {

/// @enum TransmissionMode
/// @brief Operating mode for gear change control.
enum class TransmissionMode : std::uint8_t {
    Manual = 0U,    ///< Driver controls gear selection directly.
    Sequential = 1U,///< Paddle up/down with automated torque cut and rev match.
    Automatic = 2U  ///< Fully automated shift scheduler based on engine speed.
};

/// @enum ShiftState
/// @brief Phase state during a gear change transition.
enum class ShiftState : std::uint8_t {
    InGear = 0U,       ///< Normal power transmission.
    Disengaging = 1U,  ///< Torque ramping down prior to ratio change.
    InNeutral = 2U,    ///< Ratio disengaged; gearbox in neutral.
    Synchronizing = 3U,///< Throttle blip rev-matching active.
    Engaging = 4U      ///< Torque ramping back to 100%.
};

/// @struct TransmissionConfig
/// @brief Gear ratios, shift durations, and shift thresholds.
struct TransmissionConfig {
    static constexpr std::size_t kMaxGears { 8 };

    std::size_t forwardGears { 6 }; ///< Number of forward gears (<= kMaxGears).
    std::array<double, kMaxGears> gearRatios { 3.62, 2.19, 1.52, 1.18, 0.95, 0.78, 0.0, 0.0 };
    double reverseRatio { -3.45 }; ///< Reverse gear ratio.
    double finalDriveRatio { 3.73 }; ///< Differential final drive ratio.
    double efficiency { 0.96 }; ///< Mechanical gear meshing efficiency in [0..1].
    double shiftDuration { 0.12 }; ///< Total shift time in seconds.
    double upshiftRpm { 6200.0 }; ///< Automatic upshift RPM threshold.
    double downshiftRpm { 2400.0 }; ///< Automatic downshift RPM threshold.
    TransmissionMode mode { TransmissionMode::Sequential }; ///< Operating mode.
};

/// @struct TransmissionOutput
/// @brief Instantaneous state of transmission and propeller shaft.
struct TransmissionOutput {
    double propTorque { 0.0 }; ///< Output torque to differential in N*m.
    double inputSpeed { 0.0 }; ///< Transmission input shaft speed in rad/s.
    double currentRatio { 0.0 }; ///< Active mechanical ratio.
    std::int32_t activeGear { 1 }; ///< Engaged gear index (-1 = Rev, 0 = N, 1..N = Fwd).
    ShiftState shiftPhase { ShiftState::InGear }; ///< Current shift phase.
    double shiftProgress { 1.0 }; ///< Shift progression [0.0 = start, 1.0 = done].
    bool isTorqueCut { false }; ///< True if ignition/fuel cut should be asserted.
    bool isRevBlip { false }; ///< True if throttle blip should be asserted.
};

/// @class Transmission2D
/// @brief Models multi-ratio mechanical or automated vehicle transmission.
class Transmission2D {
public:
    /// @brief Default constructor using baseline 6-speed sequential transmission.
    Transmission2D() noexcept;

    /// @brief Parameterized constructor using custom transmission configuration.
    /// @param[in] config Gearbox ratios and shift durations.
    explicit Transmission2D(const TransmissionConfig& config) noexcept;

    /// @brief Advances transmission simulation step.
    /// @param[in] clutchTorque Input torque from clutch in N*m.
    /// @param[in] propShaftSpeed Output propeller shaft speed in rad/s.
    /// @param[in] engineRpm Current engine speed in RPM for auto-shift logic.
    /// @param[in] dt Timestep duration in seconds.
    /// @return Evaluated transmission output state.
    [[nodiscard]] TransmissionOutput step(double clutchTorque,
                                          double propShaftSpeed,
                                          double engineRpm,
                                          double dt) noexcept;

    /// @brief Resets transmission to initial state.
    /// @param[in] initialGear Starting gear index (default 1).
    void reset(std::int32_t initialGear = 1) noexcept;

    /// @brief Requests a sequential upshift.
    /// @return True if upshift sequence commenced.
    bool shiftUp() noexcept;

    /// @brief Requests a sequential downshift.
    /// @return True if downshift sequence commenced.
    bool shiftDown() noexcept;

    /// @brief Selects a specific target gear directly.
    /// @param[in] gear Target gear index (-1 to forwardGears).
    /// @return True if shift accepted.
    bool selectGear(std::int32_t gear) noexcept;

    /// @brief Sets transmission operating mode.
    /// @param[in] mode Operating mode enum.
    void setMode(TransmissionMode mode) noexcept;

    /// @brief Queries current active gear index.
    /// @return Active gear (-1 = Rev, 0 = N, 1..N = Fwd).
    [[nodiscard]] std::int32_t getGear() const noexcept;

    /// @brief Queries whether a shift transition is currently executing.
    /// @return True if shifting.
    [[nodiscard]] bool isShifting() const noexcept;

    /// @brief Queries mechanical ratio for a given gear index.
    /// @param[in] gear Gear index (-1 to forwardGears).
    /// @return Ratio value (0.0 for neutral).
    [[nodiscard]] double getRatio(std::int32_t gear) const noexcept;

    /// @brief Queries active mechanical ratio.
    /// @return Active ratio value.
    [[nodiscard]] double getCurrentRatio() const noexcept;

private:
    TransmissionConfig config_ {};
    std::int32_t currentGear_ { 1 };
    std::int32_t targetGear_ { 1 };
    ShiftState shiftState_ { ShiftState::InGear };
    double shiftTimer_ { 0.0 };
};

} // namespace vehicle_sim::dynamics
