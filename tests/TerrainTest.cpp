#include <gtest/gtest.h>

#include "libs/VehicleSimCore/terrain/SurfaceMaterial.h"

namespace vehicle_sim::testing {

TEST(SurfaceMaterialTest, DefaultConstructionIsTarmac)
{
    const terrain::SurfaceMaterial mat {};
    EXPECT_EQ(mat.type, terrain::MaterialType::Tarmac);
    EXPECT_DOUBLE_EQ(mat.friction, 1.0);
    EXPECT_DOUBLE_EQ(mat.rollingDrag, 1.0);
    EXPECT_DOUBLE_EQ(mat.restitution, 0.10);
    EXPECT_FALSE(mat.isLiquid);
}

TEST(SurfaceMaterialTest, PresetRetrievalAndValidation)
{
    const auto tarmac = terrain::SurfaceMaterial::getPreset(terrain::MaterialType::Tarmac);
    EXPECT_EQ(tarmac.type, terrain::MaterialType::Tarmac);
    EXPECT_DOUBLE_EQ(tarmac.friction, 1.0);
    EXPECT_DOUBLE_EQ(tarmac.rollingDrag, 1.0);
    EXPECT_FALSE(tarmac.isLiquid);
    EXPECT_EQ(terrain::SurfaceMaterial::getName(terrain::MaterialType::Tarmac), "Tarmac");

    const auto ice = terrain::SurfaceMaterial::getPreset(terrain::MaterialType::Ice);
    EXPECT_EQ(ice.type, terrain::MaterialType::Ice);
    EXPECT_LT(ice.friction, tarmac.friction);
    EXPECT_DOUBLE_EQ(ice.friction, 0.10);
    EXPECT_FALSE(ice.isLiquid);
    EXPECT_EQ(terrain::SurfaceMaterial::getName(terrain::MaterialType::Ice), "Ice");

    const auto mud = terrain::SurfaceMaterial::getPreset(terrain::MaterialType::Mud);
    EXPECT_EQ(mud.type, terrain::MaterialType::Mud);
    EXPECT_GT(mud.rollingDrag, tarmac.rollingDrag);
    EXPECT_TRUE(mud.isLiquid);
    EXPECT_EQ(terrain::SurfaceMaterial::getName(terrain::MaterialType::Mud), "Mud");

    const auto oilSlick = terrain::SurfaceMaterial::getPreset(terrain::MaterialType::OilSlick);
    EXPECT_DOUBLE_EQ(oilSlick.friction, 0.05);
    EXPECT_EQ(terrain::SurfaceMaterial::getName(terrain::MaterialType::OilSlick), "OilSlick");

    const auto water = terrain::SurfaceMaterial::getPreset(terrain::MaterialType::Water);
    EXPECT_TRUE(water.isLiquid);
    EXPECT_GT(water.rollingDrag, 3.0);
    EXPECT_EQ(terrain::SurfaceMaterial::getName(terrain::MaterialType::Water), "Water");
}

TEST(SurfaceMaterialTest, AllPresetsCovered)
{
    const std::array<terrain::MaterialType, 10> types { terrain::MaterialType::Tarmac, terrain::MaterialType::Concrete,
        terrain::MaterialType::Dirt, terrain::MaterialType::Gravel, terrain::MaterialType::Mud,
        terrain::MaterialType::Ice, terrain::MaterialType::Snow, terrain::MaterialType::OilSlick,
        terrain::MaterialType::Water, terrain::MaterialType::Grass };

    for (const auto type : types) {
        const auto mat = terrain::SurfaceMaterial::getPreset(type);
        EXPECT_EQ(mat.type, type);
        EXPECT_GT(mat.friction, 0.0);
        EXPECT_GT(mat.rollingDrag, 0.0);
        EXPECT_GE(mat.restitution, 0.0);
        EXPECT_NE(terrain::SurfaceMaterial::getName(type), "Unknown");
    }
}

} // namespace vehicle_sim::testing
