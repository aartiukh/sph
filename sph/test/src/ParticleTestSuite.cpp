/**
 * @file ParticleTestSuite.h
 * @author Anton Artyukh (artyukhanton@gmail.com)
 * @date Created May 21, 2017
 **/

#include "ParticleTestSuite.h"

#include "Particle.h"

#include <gtest/gtest.h>

namespace SPHSDK
{
namespace TestEnvironment
{

void ParticleTestSuite::particleIsValid()
{
    Particle particle(SPHAlgorithms::Point3F(5.f, -6.f, 1.f), 0.1f);

    EXPECT_FLOAT_EQ(5.f, particle.position.x);
    EXPECT_FLOAT_EQ(-6.f, particle.position.y);
    EXPECT_FLOAT_EQ(1.f, particle.position.z);
    EXPECT_FLOAT_EQ(0.1f, particle.radius);
    EXPECT_FLOAT_EQ(0.f, particle.velocity.x);
    EXPECT_FLOAT_EQ(0.f, particle.velocity.y);
    EXPECT_FLOAT_EQ(0.f, particle.velocity.z);
    EXPECT_FLOAT_EQ(0.f, particle.density);
    EXPECT_FLOAT_EQ(0.f, particle.pressure);
}

} // namespace TestEnvironment
} // namespace SPHSDK

using namespace SPHSDK::TestEnvironment;

TEST(ParticleTestSuite, particleIsValid)
{
    ParticleTestSuite::particleIsValid();
}
