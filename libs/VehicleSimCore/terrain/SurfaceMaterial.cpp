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
        mat.soil.isDeformable = false;
        mat.soil.maxPlasticDepth = 0.0;
        break;

    case MaterialType::Concrete:
        mat.friction = 1.05;
        mat.rollingDrag = 0.95;
        mat.restitution = 0.12;
        mat.isLiquid = false;
        mat.soil.isDeformable = false;
        mat.soil.maxPlasticDepth = 0.0;
        break;

    case MaterialType::Dirt:
        mat.friction = 0.75;
        mat.rollingDrag = 1.40;
        mat.restitution = 0.08;
        mat.isLiquid = false;
        mat.soil.isDeformable = true;
        mat.soil.cohesionModulus = 30000.0;
        mat.soil.frictionModulus = 600000.0;
        mat.soil.sinkageExponent = 0.9;
        mat.soil.maxPlasticDepth = 0.20;
        mat.soil.plasticRatio = 0.85;
        mat.soil.recoveryTime = 0.0;
        mat.soil.compactionRatio = 0.45;
        mat.soil.bulkDensity = 1700.0;
        mat.soil.cohesion = 2500.0;
        mat.soil.frictionAngle = 0.50;
        mat.soil.hardeningRate = 2.0;
        break;

    case MaterialType::Gravel:
        mat.friction = 0.60;
        mat.rollingDrag = 2.00;
        mat.restitution = 0.05;
        mat.isLiquid = false;
        mat.soil.isDeformable = true;
        mat.soil.cohesionModulus = 10000.0;
        mat.soil.frictionModulus = 1500000.0;
        mat.soil.sinkageExponent = 1.0;
        mat.soil.maxPlasticDepth = 0.10;
        mat.soil.plasticRatio = 0.95;
        mat.soil.recoveryTime = 0.0;
        mat.soil.compactionRatio = 0.30;
        mat.soil.bulkDensity = 1900.0;
        mat.soil.cohesion = 500.0;
        mat.soil.frictionAngle = 0.65;
        mat.soil.hardeningRate = 2.5;
        break;

    case MaterialType::Mud:
        mat.friction = 0.40;
        mat.rollingDrag = 3.50;
        mat.restitution = 0.02;
        mat.isLiquid = true;
        mat.soil.isDeformable = true;
        mat.soil.cohesionModulus = 5000.0;
        mat.soil.frictionModulus = 150000.0;
        mat.soil.sinkageExponent = 0.7;
        mat.soil.maxPlasticDepth = 0.35;
        mat.soil.plasticRatio = 0.40;
        mat.soil.recoveryTime = 6.0;
        mat.soil.compactionRatio = 0.15;
        mat.soil.bulkDensity = 1500.0;
        mat.soil.cohesion = 1200.0;
        mat.soil.frictionAngle = 0.30;
        mat.soil.hardeningRate = 1.5;
        break;

    case MaterialType::Ice:
        mat.friction = 0.10;
        mat.rollingDrag = 0.50;
        mat.restitution = 0.15;
        mat.isLiquid = false;
        mat.soil.isDeformable = false;
        mat.soil.maxPlasticDepth = 0.0;
        break;

    case MaterialType::Snow:
        mat.friction = 0.35;
        mat.rollingDrag = 1.80;
        mat.restitution = 0.05;
        mat.isLiquid = false;
        mat.soil.isDeformable = true;
        mat.soil.cohesionModulus = 3000.0;
        mat.soil.frictionModulus = 80000.0;
        mat.soil.sinkageExponent = 1.1;
        mat.soil.maxPlasticDepth = 0.40;
        mat.soil.plasticRatio = 0.90;
        mat.soil.recoveryTime = 0.0;
        mat.soil.compactionRatio = 0.80;
        mat.soil.bulkDensity = 400.0;
        mat.soil.cohesion = 800.0;
        mat.soil.frictionAngle = 0.35;
        mat.soil.hardeningRate = 3.0;
        break;

    case MaterialType::OilSlick:
        mat.friction = 0.05;
        mat.rollingDrag = 0.60;
        mat.restitution = 0.05;
        mat.isLiquid = false;
        mat.soil.isDeformable = false;
        mat.soil.maxPlasticDepth = 0.0;
        break;

    case MaterialType::Water:
        mat.friction = 0.25;
        mat.rollingDrag = 4.00;
        mat.restitution = 0.01;
        mat.isLiquid = true;
        mat.soil.isDeformable = false;
        mat.soil.maxPlasticDepth = 0.0;
        break;

    case MaterialType::Grass:
        mat.friction = 0.70;
        mat.rollingDrag = 1.60;
        mat.restitution = 0.08;
        mat.isLiquid = false;
        mat.soil.isDeformable = true;
        mat.soil.cohesionModulus = 35000.0;
        mat.soil.frictionModulus = 800000.0;
        mat.soil.sinkageExponent = 0.95;
        mat.soil.maxPlasticDepth = 0.15;
        mat.soil.plasticRatio = 0.70;
        mat.soil.recoveryTime = 12.0;
        mat.soil.compactionRatio = 0.50;
        mat.soil.bulkDensity = 1600.0;
        mat.soil.cohesion = 3000.0;
        mat.soil.frictionAngle = 0.45;
        mat.soil.hardeningRate = 2.0;
        break;

    case MaterialType::Sand:
        mat.friction = 0.45;
        mat.rollingDrag = 2.40;
        mat.restitution = 0.03;
        mat.isLiquid = false;
        mat.soil.isDeformable = true;
        mat.soil.cohesionModulus = 1500.0;
        mat.soil.frictionModulus = 450000.0;
        mat.soil.sinkageExponent = 1.1;
        mat.soil.maxPlasticDepth = 0.30;
        mat.soil.plasticRatio = 0.90;
        mat.soil.recoveryTime = 0.0;
        mat.soil.compactionRatio = 0.35;
        mat.soil.bulkDensity = 1600.0;
        mat.soil.cohesion = 1000.0;
        mat.soil.frictionAngle = 0.60;
        mat.soil.hardeningRate = 2.0;
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
    case MaterialType::Sand:
        return "Sand";
    }

    return "Unknown";
}

} // namespace vehicle_sim::terrain
