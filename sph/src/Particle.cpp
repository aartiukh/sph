/**
* @file Particle.cpp
* @author Anton Artyukh (artyukhanton@gmail.com)
* @date Created May 21, 2017
**/

#include "Particle.h"

namespace SPHSDK
{

Particle::Particle() :
    radius(0.0),
    density(0.0),
    pressure(0.0),
    mass(0.0),
    supportRadius(0.0),
    position(SPHAlgorithms::Point3F()),
    velocity(SPHAlgorithms::Point3F()),
    acceleration(SPHAlgorithms::Point3F()),
    fGravity(SPHAlgorithms::Point3F()),
    fSurfaceTension(SPHAlgorithms::Point3F()),
    fViscosity(SPHAlgorithms::Point3F()),
    fPressure(SPHAlgorithms::Point3F()),
    fExternal(SPHAlgorithms::Point3F()),
    fInternal(SPHAlgorithms::Point3F()),
    fTotal(SPHAlgorithms::Point3F())
{
}

Particle::Particle(const SPHAlgorithms::Point3F & position, float radius) :
    radius(radius),
    density(0.0),
    pressure(0.0),
    mass(0.0),
    supportRadius(0.0),
    position(position),
    velocity(SPHAlgorithms::Point3F()),
    acceleration(SPHAlgorithms::Point3F()),
    fGravity(SPHAlgorithms::Point3F()),
    fSurfaceTension(SPHAlgorithms::Point3F()),
    fViscosity(SPHAlgorithms::Point3F()),
    fPressure(SPHAlgorithms::Point3F()),
    fExternal(SPHAlgorithms::Point3F()),
    fInternal(SPHAlgorithms::Point3F()),
    fTotal(SPHAlgorithms::Point3F())
{
}

} // SPHSDK
