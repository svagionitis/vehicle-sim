#include "Version.h"

namespace vehicle_sim {

[[nodiscard]] std::string_view getVersionString() noexcept {
    return "0.1.0";
}

} // namespace vehicle_sim
