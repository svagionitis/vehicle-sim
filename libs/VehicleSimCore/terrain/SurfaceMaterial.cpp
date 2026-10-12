#include "SurfaceMaterial.h"

namespace vehicle_sim::terrain {

SurfaceMaterial SurfaceMaterial::getPreset(MaterialType materialType) noexcept
{
    SurfaceMaterial mat {};
    mat.type = materialType;

    switch (materialType) {
    case MaterialType::Tarmac:
        mat.friction = 1.0;
        mat.rollingDrag = 1.0;
        mat.restitution = 0.10;
        mat.isLiquid = false;
        break;

    case MaterialType::Concrete:
        mat.friction = 1.05;
        mat.rollingDrag = 0.95;
        mat.restitution = 0.12;
        mat.isLiquid = false;
        break;

    case MaterialType::Dirt:
        mat.friction = 0.75;
        mat.rollingDrag = 1.40;
        mat.restitution = 0.08;
        mat.isLiquid = false;
        break;

    case MaterialType::Gravel:
        mat.friction = 0.60;
        mat.rollingDrag = 2.00;
        mat.restitution = 0.05;
        mat.isLiquid = false;
        break;

    case MaterialType::Mud:
        mat.friction = 0.40;
        mat.rollingDrag = 3.50;
        mat.restitution = 0.02;
        mat.isLiquid = true;
        break;

    case MaterialType::Ice:
        mat.friction = 0.10;
        mat.rollingDrag = 0.50;
        mat.restitution = 0.15;
        mat.isLiquid = false;
        break;

    case MaterialType::Snow:
        mat.friction = 0.35;
        mat.rollingDrag = 1.80;
        mat.restitution = 0.05;
        mat.isLiquid = false;
        break;

    case MaterialType::OilSlick:
        mat.friction = 0.05;
        mat.rollingDrag = 0.60;
        mat.restitution = 0.05;
        mat.isLiquid = false;
        break;

    case MaterialType::Water:
        mat.friction = 0.25;
        mat.rollingDrag = 4.00;
        mat.restitution = 0.01;
        mat.isLiquid = true;
        break;

    case MaterialType::Grass:
        mat.friction = 0.70;
        mat.rollingDrag = 1.60;
        mat.restitution = 0.08;
        mat.isLiquid = false;
        break;
    }

    return mat;
}

std::string_view SurfaceMaterial::getName(MaterialType materialType) noexcept
{
    switch (materialType) {
    case MaterialType::Tarmac:
        return "Tarmac";
    case MaterialType::Concrete:
        return "Concrete";
    case MaterialType::Dirt:
        return "Dirt";
    case MaterialType::Gravel:
        return "Gravel";
    case MaterialType::Mud:
        return "Mud";
    case MaterialType::Ice:
        return "Ice";
    case MaterialType::Snow:
        return "Snow";
    case MaterialType::OilSlick:
        return "OilSlick";
    case MaterialType::Water:
        return "Water";
    case MaterialType::Grass:
        return "Grass";
    }

    return "Unknown";
}

} // namespace vehicle_sim::terrain
