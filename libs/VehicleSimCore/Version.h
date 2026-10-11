#pragma once

#include <cstdint>
#include <string_view>

namespace vehicle_sim {

/// @brief Major version component of VehicleSimCore.
inline constexpr std::uint32_t kVersionMajor{0U};

/// @brief Minor version component of VehicleSimCore.
inline constexpr std::uint32_t kVersionMinor{1U};

/// @brief Patch version component of VehicleSimCore.
inline constexpr std::uint32_t kVersionPatch{0U};

/// @brief Retrieves the major version number.
/// @return Major version integer.
[[nodiscard]] constexpr std::uint32_t getVersionMajor() noexcept {
    return kVersionMajor;
}

/// @brief Retrieves the minor version number.
/// @return Minor version integer.
[[nodiscard]] constexpr std::uint32_t getVersionMinor() noexcept {
    return kVersionMinor;
}

/// @brief Retrieves the patch version number.
/// @return Patch version integer.
[[nodiscard]] constexpr std::uint32_t getVersionPatch() noexcept {
    return kVersionPatch;
}

/// @brief Retrieves the full semantic version string.
/// @return Version string in format "MAJOR.MINOR.PATCH".
[[nodiscard]] std::string_view getVersionString() noexcept;

} // namespace vehicle_sim
