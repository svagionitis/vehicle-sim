#include <gtest/gtest.h>

#include "libs/VehicleSimCore/Version.h"

namespace vehicle_sim::testing {

TEST(SanityTest, VersionConstantsMatch) {
    EXPECT_EQ(getVersionMajor(), 0U);
    EXPECT_EQ(getVersionMinor(), 1U);
    EXPECT_EQ(getVersionPatch(), 0U);
    EXPECT_EQ(getVersionString(), "0.1.0");
}

} // namespace vehicle_sim::testing
