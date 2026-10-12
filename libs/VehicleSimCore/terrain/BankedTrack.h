#pragma once

#include <cmath>
#include <cstddef>
#include <cstdint>

#include "libs/VehicleSimCore/math/Vec2.h"
#include "libs/VehicleSimCore/terrain/SurfaceMap2D.h"

namespace vehicle_sim::terrain {

/// @enum TrackCamberType
/// @brief Geometric cross-sectional profile algorithm across track ribbon.
enum class TrackCamberType : std::uint8_t {
    Flat, ///< Uniform zero transversal slope.
    Superelevation, ///< Linear inward banking balancing curve centripetal acceleration.
    Crowned, ///< Convex apex on straights for drainage, transitioning to superelevation in corners.
    ParabolicBowl ///< Progressive quadratic cross-slope steepening toward the outer barrier.
};

/// @struct CamberProfileSample
/// @brief Complete evaluated cross-section camber sample at track coordinates (s, w).
struct CamberProfileSample {
    double elevationOffset { 0.0 }; ///< Elevation delta Delta h relative to centerline in meters.
    double bankAngle { 0.0 }; ///< Transversal inclination angle theta in radians.
    double lateralSlope { 0.0 }; ///< First derivative dh/dw (gradient across ribbon).
    math::Vec2 lateralNormal { 0.0, 0.0 }; ///< Unit lateral vector pointing to left boundary.
    SurfaceNormal3D surfaceNormal { 0.0, 0.0, 1.0 }; ///< Tilted 3D upward surface normal.
};

/// @struct BankedTrackConfig
/// @brief Configuration parameters for procedural track banking and camber.
struct BankedTrackConfig {
    bool enabled { true }; ///< True if procedural banking is active.
    TrackCamberType camberType { TrackCamberType::Superelevation }; ///< Cross-section shape.
    double designSpeed { 25.0 }; ///< Target neutral-steer cornering speed in m/s (~90 km/h).
    double maxBankAngle { 0.40 }; ///< Maximum allowed bank angle in radians (~22.9 deg).
    double crownSlope { 0.02 }; ///< Drainage cross-fall on straights (e.g. 2% slope).
    double transitionLength { 30.0 }; ///< Superelevation runoff easement length in meters.
    double bowlCurvature { 0.05 }; ///< Quadratic coefficient for ParabolicBowl profile.
};

/// @class BankedTrackProfile
/// @brief Evaluates continuous transversal banking and cross-section camber along track ribbons.
/// @details Analyzes centerline curvature, designs equilibrium superelevation, computes
/// smooth runoff easements, and provides cross-section elevation and surface normal samples.
class BankedTrackProfile {
public:
    /// @brief Default constructor with default configuration.
    BankedTrackProfile() noexcept = default;

    /// @brief Constructs profile with specific configuration.
    /// @param[in] config Banking and camber parameters.
    explicit BankedTrackProfile(const BankedTrackConfig& config) noexcept;

    /// @brief Sets banking configuration parameters.
    /// @param[in] config New configuration.
    void setConfig(const BankedTrackConfig& config) noexcept;

    /// @brief Gets active banking configuration.
    /// @return Const reference to configuration.
    [[nodiscard]] const BankedTrackConfig& getConfig() const noexcept;

    /// @brief Computes theoretical equilibrium bank angle for a given curvature.
    /// @param[in] curvature Signed road curvature kappa in 1/m.
    /// @return Equilibrium bank angle theta in radians clamped to [-maxBankAngle, maxBankAngle].
    [[nodiscard]] double computeBankAngle(double curvature) const noexcept;

    /// @brief Evaluates cross-section camber elevation offset and normals.
    /// @param[in] bankAngle Local transversal bank angle in radians.
    /// @param[in] lateralOffset Signed offset w from centerline in meters (positive = left).
    /// @param[in] halfWidth Half-width of the road ribbon in meters.
    /// @param[in] tangent Unit forward tangent vector along track.
    /// @return Evaluated CamberProfileSample.
    [[nodiscard]] CamberProfileSample evaluateCamber(
        double bankAngle, double lateralOffset, double halfWidth, const math::Vec2& tangent) const noexcept;

    /// @brief Computes zero-slip neutral steer speed on a banked curve.
    /// @param[in] bankAngle Bank angle in radians.
    /// @param[in] radius Radius of curvature in meters.
    /// @param[in] gravity Gravitational acceleration in m/s^2 (default 9.81).
    /// @return Neutral cornering speed in m/s (0.0 if flat or invalid).
    [[nodiscard]] static double computeNeutralSpeed(double bankAngle, double radius, double gravity = 9.81) noexcept;

    /// @brief Computes maximum cornering speed before outward slide on banked curve.
    /// @param[in] bankAngle Bank angle in radians.
    /// @param[in] radius Radius of curvature in meters.
    /// @param[in] friction Surface tire friction coefficient mu.
    /// @param[in] gravity Gravitational acceleration in m/s^2 (default 9.81).
    /// @return Maximum cornering speed in m/s.
    [[nodiscard]] static double computeMaxSpeed(
        double bankAngle, double radius, double friction, double gravity = 9.81) noexcept;

private:
    BankedTrackConfig mConfig { };
};

} // namespace vehicle_sim::terrain
