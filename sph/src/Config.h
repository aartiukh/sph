#ifndef CONFIG_H_73C34465A6ED4DB9B9F2F4C3937BF5DC
#define CONFIG_H_73C34465A6ED4DB9B9F2F4C3937BF5DC

#include "algorithms/src/Point.h"

#include <cstddef>

namespace SPHSDK
{
struct Config
{
    static const size_t ParticlesNumber;
    static const float ParticleRadius;

    static const float WaterDensity;
    static const float WaterStiffness;
    static const float WaterViscosity;
    static const float WaterThreshold;
    static const float WaterParticleMass;
    static const float WaterSupportRadius;
    static const float WaterSurfaceTension;

    static const SPHAlgorithms::Point3F InitialGravitationalAcceleration;
    static SPHAlgorithms::Point3F GravitationalAcceleration;
    static const SPHAlgorithms::Point3F InitialVelocity;
    static const float CollisionVelocityMultiplier;

    static const float SpeedTreshold;

    static const float CubeSize;

}; //Config
} //SPHSDK

#endif // CONFIG_H_73C34465A6ED4DB9B9F2F4C3937BF5DC
