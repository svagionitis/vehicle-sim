#pragma once

#include <cstdint>

namespace vehicle_sim::dynamics {

/// @enum DifferentialType
/// @brief Differential mechanical locking policy.
enum class DifferentialType : std::uint8_t {
    Open = 0U,                   ///< Unconstrained wheel speeds; 50/50 torque split.
    LimitedSlipViscous = 1U,     ///< Viscous coupling transfer proportional to delta w.
    LimitedSlipSalisbury = 2U,   ///< Clutch-pack LSD with asymmetric drive/coast ramps.
    Torsen = 3U,                 ///< Crossed-axis helical gear torque-biasing ratio.
    LockedSpool = 4U,            ///< Rigid 1:1 axle locking both wheel speeds together.
    ActiveTorqueVectoring = 5U   ///< Electronically commanded yaw-moment bias.
};

/// @struct DifferentialConfig
/// @brief Mechanical parameters for differential models.
struct DifferentialConfig {
    DifferentialType type { DifferentialType::LimitedSlipSalisbury };
    double preloadTorque { 40.0 }; ///< Salisbury static preload in N*m.
    double driveRampRatio { 0.65 }; ///< Salisbury drive lock coefficient in [0..1].
    double coastRampRatio { 0.35 }; ///< Salisbury coast/overrun lock coefficient in [0..1].
    double viscousStiffness { 12.0 }; ///< Viscous coupling stiffness in N*m*s/rad.
    double torsenBiasRatio { 3.5 }; ///< Torsen torque bias ratio (T_high / T_low).
    double finalDriveEfficiency { 0.98 }; ///< Final drive gear meshing efficiency in [0..1].
};

/// @struct DifferentialOutput
/// @brief Torque distribution delivered to left and right drive wheels.
struct DifferentialOutput {
    double leftTorque { 0.0 }; ///< Net drive torque delivered to left wheel in N*m.
    double rightTorque { 0.0 }; ///< Net drive torque delivered to right wheel in N*m.
    double carrierSpeed { 0.0 }; ///< Differential carrier angular velocity in rad/s.
    double deltaSpeed { 0.0 }; ///< Speed difference (w_left - w_right) in rad/s.
    double lockingTorque { 0.0 }; ///< Transferred cross-axle locking torque in N*m.
};

/// @class Differential2D
/// @brief Models axle and center differentials with multiple locking mechanisms.
class Differential2D {
public:
    /// @brief Default constructor using Salisbury LSD parameters.
    Differential2D() noexcept;

    /// @brief Parameterized constructor using custom differential configuration.
    /// @param[in] config Differential configuration struct.
    explicit Differential2D(const DifferentialConfig& config) noexcept;

    /// @brief Computes torque distribution across left and right wheels.
    /// @param[in] inputTorque Propeller shaft input torque in N*m.
    /// @param[in] leftSpeed Left wheel hub angular velocity in rad/s.
    /// @param[in] rightSpeed Right wheel hub angular velocity in rad/s.
    /// @param[in] yawBiasCommand Optional active torque vectoring bias [-1.0, 1.0].
    /// @return Evaluated differential torque distribution.
    [[nodiscard]] DifferentialOutput step(double inputTorque,
                                          double leftSpeed,
                                          double rightSpeed,
                                          double yawBiasCommand = 0.0) const noexcept;

    /// @brief Sets differential locking policy type.
    /// @param[in] type DifferentialType enum.
    void setType(DifferentialType type) noexcept;

    /// @brief Queries current differential type.
    /// @return Active DifferentialType.
    [[nodiscard]] DifferentialType getType() const noexcept;

    /// @brief Queries active differential configuration.
    /// @return Reference to active DifferentialConfig.
    [[nodiscard]] const DifferentialConfig& getConfig() const noexcept;

    /// @brief Updates differential configuration parameters.
    /// @param[in] config New configuration struct.
    void setConfig(const DifferentialConfig& config) noexcept;

private:
    DifferentialConfig config_ {};
};

} // namespace vehicle_sim::dynamics
