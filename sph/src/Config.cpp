//
//  Config.cpp
//
//  Created by Oleksii Shabalin on 11/3/18.
//

#include "Config.h"

namespace SPHSDK
{
    const size_t Config::ParticlesNumber = 6000;
    const float Config::ParticleRadius = 0.015f;

    const float Config::WaterDensity = 998.29f;
    const float Config::WaterStiffness = 3.0f;
    const float Config::WaterViscosity = 3.5f;
    const float Config::WaterThreshold = 7.065f;
    const float Config::WaterParticleMass = 0.02f;
    const float Config::WaterSupportRadius = 0.1f;
    const float Config::WaterSurfaceTension = 0.0728f;

    const SPHAlgorithms::Point3F Config::InitialGravitationalAcceleration(0.f, 0.f, -9.82f);
    SPHAlgorithms::Point3F Config::GravitationalAcceleration(0.f, 0.f, -9.82f);
    const SPHAlgorithms::Point3F Config::InitialVelocity(0.f, 0.f, 0.f);
    const float Config::CollisionVelocityMultiplier = -0.5f;

    const float Config::SpeedTreshold = 3.f;

    const float Config::CubeSize = 3.f;
} //SPHSDK
