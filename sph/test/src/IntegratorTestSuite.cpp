/**
 * @file IntegratorTestSuite.cpp
 * @author Anton Artyukh (artyukhanton@gmail.com)
 * @date Created June 11, 2017
 **/

#include "IntegratorTestSuite.h"

#include "Integrator.h"

#include <gtest/gtest.h>

namespace SPHSDK
{
namespace TestEnvironment
{

void IntegratorTestSuite::oneParticleWithZeroVelocity()
{
    ParticleVect particles = {Particle(SPHAlgorithms::Point3F(0.f, 1.f, 1.f), 0.1f)};
    particles[0].density = 0.5f;
    particles[0].fTotal = SPHAlgorithms::Point3F(0.25f, 0.25f, 0.25f);
    particles[0].acceleration = SPHAlgorithms::Point3F(0.1f, 0.1f, 0.1f);

    Integrator::integrate(0.01f, particles);

    EXPECT_FLOAT_EQ(0.003f, particles[0].velocity.x);
    EXPECT_FLOAT_EQ(0.003f, particles[0].velocity.y);
    EXPECT_FLOAT_EQ(0.003f, particles[0].velocity.z);
    EXPECT_FLOAT_EQ(5.0e-06f, particles[0].position.x);
    EXPECT_FLOAT_EQ(1.000005f, particles[0].position.y);
    EXPECT_FLOAT_EQ(1.000005f, particles[0].position.z);
}

void IntegratorTestSuite::oneParticleWithZeroDensity()
{
    ParticleVect particles = {Particle(SPHAlgorithms::Point3F(0.f, 1.f, 1.f), 0.1f)};
    particles[0].fTotal = SPHAlgorithms::Point3F(0.25f, 0.25f, 0.25f);
    particles[0].acceleration = SPHAlgorithms::Point3F(0.1f, 0.1f, 0.1f);

    Integrator::integrate(0.01f, particles);

    EXPECT_FLOAT_EQ(0.1f, particles[0].acceleration.x);
    EXPECT_FLOAT_EQ(0.1f, particles[0].acceleration.y);
    EXPECT_FLOAT_EQ(0.1f, particles[0].acceleration.z);
    EXPECT_FLOAT_EQ(0.001f, particles[0].velocity.x);
    EXPECT_FLOAT_EQ(0.001f, particles[0].velocity.y);
    EXPECT_FLOAT_EQ(0.001f, particles[0].velocity.z);
    EXPECT_FLOAT_EQ(5.0e-06f, particles[0].position.x);
    EXPECT_FLOAT_EQ(1.000005f, particles[0].position.y);
    EXPECT_FLOAT_EQ(1.000005f, particles[0].position.z);
}

} // namespace TestEnvironment
} // namespace SPHSDK

using namespace SPHSDK::TestEnvironment;

TEST(IntegratorTestSuite, oneParticleWithZeroVelocity)
{
    IntegratorTestSuite::oneParticleWithZeroVelocity();
}

TEST(IntegratorTestSuite, oneParticleWithZeroDensity)
{
    IntegratorTestSuite::oneParticleWithZeroDensity();
}
