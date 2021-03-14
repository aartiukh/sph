/**
 * @file CollisionsTestSuite.h
 * @author Anton Artyukh (artyukhanton@gmail.com)
 * @date Created May 28, 2017
 **/

#include "CollisionsTestSuite.h"

#include "Collisions.h"
#include "algorithms/src/Area.h"

#include <gtest/gtest.h>

namespace SPHSDK
{
namespace TestEnvironment
{

void CollisionsTestSuite::twoParticleCollision()
{
    ParticleVect particleVector = {Particle(SPHAlgorithms::Point3F(0.01f, 0.01f, 0.01f), 0.1f),
                                   Particle(SPHAlgorithms::Point3F(0.09f, 0.09f, 0.09f), 0.1f)};
    particleVector[0].velocity = SPHAlgorithms::Point3F(2.0f, 2.0f, 2.0f);
    particleVector[1].velocity = SPHAlgorithms::Point3F(-2.0f, -2.0f, -2.0f);
    particleVector[0].neighbours = {1};
    particleVector[1].neighbours = {0};
    SPHAlgorithms::Volume volume(SPHAlgorithms::Cuboid(SPHAlgorithms::Point3F(0.f, 0.f, 0.f), 1.f, 1.f, 1.f));

    Collision::detectCollisions(particleVector, volume);

    EXPECT_FLOAT_EQ(0.1f, particleVector[0].position.x);
    EXPECT_FLOAT_EQ(0.1f, particleVector[0].position.y);
    EXPECT_FLOAT_EQ(0.1f, particleVector[0].position.z);
    EXPECT_FLOAT_EQ(0.1f, particleVector[1].position.x);
    EXPECT_FLOAT_EQ(0.1f, particleVector[1].position.y);
    EXPECT_FLOAT_EQ(0.1f, particleVector[1].position.z);

    EXPECT_FLOAT_EQ(-1.f, particleVector[0].velocity.x);
    EXPECT_FLOAT_EQ(-1.f, particleVector[0].velocity.y);
    EXPECT_FLOAT_EQ(-1.f, particleVector[0].velocity.z);
    EXPECT_FLOAT_EQ(1.f, particleVector[1].velocity.x);
    EXPECT_FLOAT_EQ(1.f, particleVector[1].velocity.y);
    EXPECT_FLOAT_EQ(1.f, particleVector[1].velocity.z);
}

void CollisionsTestSuite::threeParticleCollision()
{
    ParticleVect particleVector = {Particle(SPHAlgorithms::Point3F(1.f, 1.f, 1.f), 1.6f),
                                   Particle(SPHAlgorithms::Point3F(3.f, 1.f, 1.f), 1.6f),
                                   Particle(SPHAlgorithms::Point3F(2.f, 2.f, 2.f), 1.6f)};
    particleVector[0].velocity = SPHAlgorithms::Point3F(2.f, 2.f, 2.f);
    particleVector[1].velocity = SPHAlgorithms::Point3F(-2.f, 2.f, 2.f);
    particleVector[2].velocity = SPHAlgorithms::Point3F(0.f, -2.f, 0.f);
    particleVector[0].neighbours = {1, 2};
    particleVector[1].neighbours = {0, 2};
    particleVector[2].neighbours = {0, 1};

    SPHAlgorithms::Volume volume(SPHAlgorithms::Cuboid(SPHAlgorithms::Point3F(0.f, 0.f, 0.f), 4.f, 4.f, 4.f));

    Collision::detectCollisions(particleVector, volume);

    EXPECT_FLOAT_EQ(1.6f, particleVector[0].position.x);
    EXPECT_FLOAT_EQ(1.6f, particleVector[0].position.y);
    EXPECT_FLOAT_EQ(1.6f, particleVector[0].position.z);
    EXPECT_FLOAT_EQ(2.4f, particleVector[1].position.x);
    EXPECT_FLOAT_EQ(1.6f, particleVector[1].position.y);
    EXPECT_FLOAT_EQ(1.6f, particleVector[1].position.z);
    EXPECT_FLOAT_EQ(2.f, particleVector[2].position.x);
    EXPECT_FLOAT_EQ(2.f, particleVector[2].position.y);
    EXPECT_FLOAT_EQ(2.f, particleVector[2].position.z);

    EXPECT_FLOAT_EQ(-1.f, particleVector[0].velocity.x);
    EXPECT_FLOAT_EQ(-1.f, particleVector[0].velocity.y);
    EXPECT_FLOAT_EQ(-1.f, particleVector[0].velocity.z);
    EXPECT_FLOAT_EQ(1.f, particleVector[1].velocity.x);
    EXPECT_FLOAT_EQ(-1.f, particleVector[1].velocity.y);
    EXPECT_FLOAT_EQ(-1.f, particleVector[1].velocity.z);
    EXPECT_FLOAT_EQ(0.f, particleVector[2].velocity.x);
    EXPECT_FLOAT_EQ(-2.f, particleVector[2].velocity.y);
    EXPECT_FLOAT_EQ(0.f, particleVector[2].velocity.z);
}

void CollisionsTestSuite::fourParticleCollision()
{
    ParticleVect particleVector = {
        Particle(SPHAlgorithms::Point3F(1.f, 1.f, 1.f), 1.6f), Particle(SPHAlgorithms::Point3F(3.f, 1.f, 1.f), 1.6f),
        Particle(SPHAlgorithms::Point3F(1.f, 3.f, 1.f), 1.6f), Particle(SPHAlgorithms::Point3F(3.f, 3.f, 3.f), 1.6f)};
    particleVector[0].velocity = SPHAlgorithms::Point3F(2.f, 2.f, 2.f);
    particleVector[1].velocity = SPHAlgorithms::Point3F(-2.f, 2.f, 2.f);
    particleVector[2].velocity = SPHAlgorithms::Point3F(2.f, -2.f, 2.f);
    particleVector[3].velocity = SPHAlgorithms::Point3F(-2.f, -2.f, 2.f);
    particleVector[0].neighbours = {1, 2, 3};
    particleVector[1].neighbours = {0, 2, 3};
    particleVector[2].neighbours = {0, 1, 3};
    particleVector[3].neighbours = {0, 1, 2};

    SPHAlgorithms::Volume volume(SPHAlgorithms::Cuboid(SPHAlgorithms::Point3F(0.f, 0.f, 0.f), 4.f, 4.f, 4.f));

    Collision::detectCollisions(particleVector, volume);

    EXPECT_FLOAT_EQ(1.6f, particleVector[0].position.x);
    EXPECT_FLOAT_EQ(1.6f, particleVector[0].position.y);
    EXPECT_FLOAT_EQ(1.6f, particleVector[0].position.z);
    EXPECT_FLOAT_EQ(2.4f, particleVector[1].position.x);
    EXPECT_FLOAT_EQ(1.6f, particleVector[1].position.y);
    EXPECT_FLOAT_EQ(1.6f, particleVector[1].position.z);
    EXPECT_FLOAT_EQ(1.6f, particleVector[2].position.x);
    EXPECT_FLOAT_EQ(2.4f, particleVector[2].position.y);
    EXPECT_FLOAT_EQ(1.6f, particleVector[2].position.z);
    EXPECT_FLOAT_EQ(2.4f, particleVector[3].position.x);
    EXPECT_FLOAT_EQ(2.4f, particleVector[3].position.y);
    EXPECT_FLOAT_EQ(2.4f, particleVector[3].position.z);

    EXPECT_FLOAT_EQ(-1.f, particleVector[0].velocity.x);
    EXPECT_FLOAT_EQ(-1.f, particleVector[0].velocity.y);
    EXPECT_FLOAT_EQ(-1.f, particleVector[0].velocity.z);
    EXPECT_FLOAT_EQ(1.f, particleVector[1].velocity.x);
    EXPECT_FLOAT_EQ(-1.f, particleVector[1].velocity.y);
    EXPECT_FLOAT_EQ(-1.f, particleVector[1].velocity.z);
    EXPECT_FLOAT_EQ(-1.f, particleVector[2].velocity.x);
    EXPECT_FLOAT_EQ(1.f, particleVector[2].velocity.y);
    EXPECT_FLOAT_EQ(-1.f, particleVector[2].velocity.z);
    EXPECT_FLOAT_EQ(1.f, particleVector[3].velocity.x);
    EXPECT_FLOAT_EQ(1.f, particleVector[3].velocity.y);
    EXPECT_FLOAT_EQ(-1.f, particleVector[3].velocity.z);
}

void CollisionsTestSuite::twoAndTwoParticleCollision()
{
    ParticleVect particleVector = {
        Particle(SPHAlgorithms::Point3F(1.f, 1.f, 1.f), 0.55f), Particle(SPHAlgorithms::Point3F(1.5f, 1.f, 1.f), 0.55f),
        Particle(SPHAlgorithms::Point3F(1.f, 3.f, 3.f), 0.55f), Particle(SPHAlgorithms::Point3F(1.5f, 3.f, 3.f), 0.55f)};
    particleVector[0].velocity = SPHAlgorithms::Point3F(2.f, 0.f, 1.f);
    particleVector[1].velocity = SPHAlgorithms::Point3F(-2.f, 0.f, 1.f);
    particleVector[2].velocity = SPHAlgorithms::Point3F(2.f, 0.f, 1.f);
    particleVector[3].velocity = SPHAlgorithms::Point3F(-2.f, 0.f, 1.f);
    particleVector[0].neighbours = {1, 2, 3};
    particleVector[1].neighbours = {0, 2, 3};
    particleVector[2].neighbours = {0, 1, 3};
    particleVector[3].neighbours = {0, 1, 2};

    SPHAlgorithms::Volume volume(SPHAlgorithms::Cuboid(SPHAlgorithms::Point3F(0.f, 0.f, 1.f), 4.f, 4.f, 1.f));

    Collision::detectCollisions(particleVector, volume);

    EXPECT_FLOAT_EQ(2.f, particleVector[0].velocity.x);
    EXPECT_FLOAT_EQ(0.f, particleVector[0].velocity.y);
    EXPECT_FLOAT_EQ(0.25f, particleVector[0].velocity.z);
    EXPECT_FLOAT_EQ(-2.f, particleVector[1].velocity.x);
    EXPECT_FLOAT_EQ(0.f, particleVector[1].velocity.y);
    EXPECT_FLOAT_EQ(0.25f, particleVector[1].velocity.z);
    EXPECT_FLOAT_EQ(2.f, particleVector[2].velocity.x);
    EXPECT_FLOAT_EQ(0.f, particleVector[2].velocity.y);
    EXPECT_FLOAT_EQ(0.25f, particleVector[2].velocity.z);
    EXPECT_FLOAT_EQ(-2.f, particleVector[3].velocity.x);
    EXPECT_FLOAT_EQ(0.f, particleVector[3].velocity.y);
    EXPECT_FLOAT_EQ(0.25f, particleVector[3].velocity.z);
}

void CollisionsTestSuite::oneAndFiveParticleCollision()
{
    ParticleVect particleVector = {
        Particle(SPHAlgorithms::Point3F(1.f, 1.f, 1.f), 0.12f), Particle(SPHAlgorithms::Point3F(1.1f, 1.f, 1.f), 0.12f),
        Particle(SPHAlgorithms::Point3F(1.2f, 1.f, 1.f), 0.12f), Particle(SPHAlgorithms::Point3F(1.3f, 1.f, 1.f), 0.12f),
        Particle(SPHAlgorithms::Point3F(1.4f, 1.f, 1.f), 0.12f), Particle(SPHAlgorithms::Point3F(1.5f, 1.f, 1.f), 0.12f)};
    particleVector[0].velocity = SPHAlgorithms::Point3F(0.f, 0.f, 1.f);
    particleVector[1].velocity = SPHAlgorithms::Point3F(0.f, 0.f, 1.f);
    particleVector[2].velocity = SPHAlgorithms::Point3F(0.f, 0.f, 1.f);
    particleVector[3].velocity = SPHAlgorithms::Point3F(0.f, 0.f, 1.f);
    particleVector[4].velocity = SPHAlgorithms::Point3F(0.f, 0.f, 1.f);
    particleVector[5].velocity = SPHAlgorithms::Point3F(-1.f, 0.f, 1.f);
    particleVector[0].neighbours = {1};
    particleVector[1].neighbours = {0, 2};
    particleVector[2].neighbours = {1, 3};
    particleVector[3].neighbours = {2, 4};
    particleVector[4].neighbours = {3, 5};
    particleVector[5].neighbours = {4};

    SPHAlgorithms::Volume volume(SPHAlgorithms::Cuboid(SPHAlgorithms::Point3F(0.f, 0.f, 1.f), 4.f, 4.f, 1.f));

    Collision::detectCollisions(particleVector, volume);

    EXPECT_FLOAT_EQ(0.f, particleVector[0].velocity.x);
    EXPECT_FLOAT_EQ(0.f, particleVector[0].velocity.y);
    EXPECT_FLOAT_EQ(-0.5f, particleVector[0].velocity.z);
    EXPECT_FLOAT_EQ(0.f, particleVector[1].velocity.x);
    EXPECT_FLOAT_EQ(0.f, particleVector[1].velocity.y);
    EXPECT_FLOAT_EQ(-0.5f, particleVector[1].velocity.z);
    EXPECT_FLOAT_EQ(0.f, particleVector[2].velocity.x);
    EXPECT_FLOAT_EQ(0.f, particleVector[2].velocity.y);
    EXPECT_FLOAT_EQ(-0.5f, particleVector[2].velocity.z);
    EXPECT_FLOAT_EQ(0.f, particleVector[3].velocity.x);
    EXPECT_FLOAT_EQ(0.f, particleVector[3].velocity.y);
    EXPECT_FLOAT_EQ(-0.5f, particleVector[3].velocity.z);
    EXPECT_FLOAT_EQ(0.f, particleVector[4].velocity.x);
    EXPECT_FLOAT_EQ(0.f, particleVector[4].velocity.y);
    EXPECT_FLOAT_EQ(-0.5f, particleVector[4].velocity.z);
    EXPECT_FLOAT_EQ(-1.f, particleVector[5].velocity.x);
    EXPECT_FLOAT_EQ(0.f, particleVector[5].velocity.y);
    EXPECT_FLOAT_EQ(-0.5f, particleVector[5].velocity.z);
}

void CollisionsTestSuite::oneAndFourParticleCollision()
{
    ParticleVect particleVector = {
        Particle(SPHAlgorithms::Point3F(1.f, 1.f, 1.f), 0.12f), Particle(SPHAlgorithms::Point3F(1.1f, 1.f, 1.f), 0.12f),
        Particle(SPHAlgorithms::Point3F(1.2f, 1.f, 1.f), 0.12f), Particle(SPHAlgorithms::Point3F(1.3f, 1.f, 1.f), 0.12f),
        Particle(SPHAlgorithms::Point3F(1.25f, 1.1f, 1.f), 0.12f)};
    particleVector[0].velocity = SPHAlgorithms::Point3F(0.f, 0.f, 1.f);
    particleVector[1].velocity = SPHAlgorithms::Point3F(0.f, 0.f, 1.f);
    particleVector[2].velocity = SPHAlgorithms::Point3F(0.f, 0.f, 1.f);
    particleVector[3].velocity = SPHAlgorithms::Point3F(0.f, 0.f, 1.f);
    particleVector[4].velocity = SPHAlgorithms::Point3F(0.f, 5.f, 1.f);
    particleVector[0].neighbours = {1};
    particleVector[1].neighbours = {0, 2, 4};
    particleVector[2].neighbours = {1, 3, 4};
    particleVector[3].neighbours = {2};
    particleVector[4].neighbours = {1, 2};

    SPHAlgorithms::Volume volume(SPHAlgorithms::Cuboid(SPHAlgorithms::Point3F(0.f, 0.f, 1.f), 4.f, 4.f, 1.f));

    Collision::detectCollisions(particleVector, volume);

    EXPECT_FLOAT_EQ(0.f, particleVector[0].velocity.x);
    EXPECT_FLOAT_EQ(0.f, particleVector[0].velocity.y);
    EXPECT_FLOAT_EQ(-0.5f, particleVector[0].velocity.z);
    EXPECT_FLOAT_EQ(0.f, particleVector[1].velocity.x);
    EXPECT_FLOAT_EQ(0.f, particleVector[1].velocity.y);
    EXPECT_FLOAT_EQ(-0.5f, particleVector[1].velocity.z);
    EXPECT_FLOAT_EQ(0.f, particleVector[2].velocity.x);
    EXPECT_FLOAT_EQ(0.f, particleVector[2].velocity.y);
    EXPECT_FLOAT_EQ(-0.5f, particleVector[2].velocity.z);
    EXPECT_FLOAT_EQ(0.f, particleVector[3].velocity.x);
    EXPECT_FLOAT_EQ(0.f, particleVector[3].velocity.y);
    EXPECT_FLOAT_EQ(-0.5f, particleVector[3].velocity.z);
    EXPECT_FLOAT_EQ(0.f, particleVector[4].velocity.x);
    EXPECT_FLOAT_EQ(5.f, particleVector[4].velocity.y);
    EXPECT_FLOAT_EQ(-0.5f, particleVector[4].velocity.z);
}

void CollisionsTestSuite::oneAndEightParticleCollision()
{
    ParticleVect particleVector = {
        Particle(SPHAlgorithms::Point3F(1.f, 1.f, 1.f), 0.51f), Particle(SPHAlgorithms::Point3F(0.75f, 1.25f, 1.f), 0.51f),
        Particle(SPHAlgorithms::Point3F(0.5f, 1.5f, 1.f), 0.51f), Particle(SPHAlgorithms::Point3F(0.75f, 1.75f, 1.f), 0.51f),
        Particle(SPHAlgorithms::Point3F(1.f, 2.f, 1.f), 0.51f), Particle(SPHAlgorithms::Point3F(1.25f, 1.75f, 1.f), 0.51f),
        Particle(SPHAlgorithms::Point3F(1.5f, 1.5f, 1.f), 0.51f), Particle(SPHAlgorithms::Point3F(1.25f, 1.25f, 1.f), 0.51f),
        Particle(SPHAlgorithms::Point3F(1.f, 1.5f, 1.f), 0.51f)};
    particleVector[0].velocity = SPHAlgorithms::Point3F(0.f, 1.f, 1.f);
    particleVector[1].velocity = SPHAlgorithms::Point3F(1.f, 1.f, 1.f);
    particleVector[2].velocity = SPHAlgorithms::Point3F(1.f, 0.f, 1.f);
    particleVector[3].velocity = SPHAlgorithms::Point3F(1.f, -1.f, 1.f);
    particleVector[4].velocity = SPHAlgorithms::Point3F(0.f, -1.f, 1.f);
    particleVector[5].velocity = SPHAlgorithms::Point3F(-1.f, -1.f, 1.f);
    particleVector[6].velocity = SPHAlgorithms::Point3F(-1.f, 0.f, 1.f);
    particleVector[7].velocity = SPHAlgorithms::Point3F(-1.f, 1.f, 1.f);
    particleVector[8].velocity = SPHAlgorithms::Point3F(0.f, 0.f, 1.f);
    particleVector[0].neighbours = {1, 7, 8};
    particleVector[1].neighbours = {0, 2, 8};
    particleVector[2].neighbours = {1, 3, 8};
    particleVector[3].neighbours = {2, 4, 8};
    particleVector[4].neighbours = {3, 5, 8};
    particleVector[5].neighbours = {4, 6, 8};
    particleVector[6].neighbours = {5, 7, 8};
    particleVector[7].neighbours = {0, 6, 8};
    particleVector[8].neighbours = {0, 1, 2, 3, 4, 5, 6, 7};

    SPHAlgorithms::Volume volume(SPHAlgorithms::Cuboid(SPHAlgorithms::Point3F(0.f, 0.f, 0.f), 4.f, 4.f, 1.f));

    Collision::detectCollisions(particleVector, volume);

    EXPECT_FLOAT_EQ(0.f, particleVector[0].velocity.x);
    EXPECT_FLOAT_EQ(1.f, particleVector[0].velocity.y);
    EXPECT_FLOAT_EQ(0.25f, particleVector[0].velocity.z);
    EXPECT_FLOAT_EQ(1.f, particleVector[1].velocity.x);
    EXPECT_FLOAT_EQ(1.f, particleVector[1].velocity.y);
    EXPECT_FLOAT_EQ(0.25f, particleVector[1].velocity.z);
    EXPECT_FLOAT_EQ(-0.5f, particleVector[2].velocity.x);
    EXPECT_FLOAT_EQ(0.f, particleVector[2].velocity.y);
    EXPECT_FLOAT_EQ(0.25f, particleVector[2].velocity.z);
    EXPECT_FLOAT_EQ(1.f, particleVector[3].velocity.x);
    EXPECT_FLOAT_EQ(-1.f, particleVector[3].velocity.y);
    EXPECT_FLOAT_EQ(0.25f, particleVector[3].velocity.z);
    EXPECT_FLOAT_EQ(0.f, particleVector[4].velocity.x);
    EXPECT_FLOAT_EQ(-1.f, particleVector[4].velocity.y);
    EXPECT_FLOAT_EQ(0.25f, particleVector[4].velocity.z);
    EXPECT_FLOAT_EQ(-1.f, particleVector[5].velocity.x);
    EXPECT_FLOAT_EQ(-1.f, particleVector[5].velocity.y);
    EXPECT_FLOAT_EQ(0.25f, particleVector[5].velocity.z);
    EXPECT_FLOAT_EQ(-1.f, particleVector[6].velocity.x);
    EXPECT_FLOAT_EQ(0.f, particleVector[6].velocity.y);
    EXPECT_FLOAT_EQ(0.25f, particleVector[6].velocity.z);
    EXPECT_FLOAT_EQ(-1.f, particleVector[7].velocity.x);
    EXPECT_FLOAT_EQ(1.f, particleVector[7].velocity.y);
    EXPECT_FLOAT_EQ(0.25f, particleVector[7].velocity.z);
    EXPECT_FLOAT_EQ(0.f, particleVector[8].velocity.x);
    EXPECT_FLOAT_EQ(0.f, particleVector[8].velocity.y);
    EXPECT_FLOAT_EQ(0.25f, particleVector[8].velocity.z);
}

void CollisionsTestSuite::oneOnBoundaryParticleCollision()
{
    ParticleVect particleVector = {Particle(SPHAlgorithms::Point3F(-9.83f, -0.003716f, -1.f), 0.1f)};
    particleVector[0].velocity = SPHAlgorithms::Point3F(-0.5f, -7.f, -1.f);
    SPHAlgorithms::Volume volume(SPHAlgorithms::Cuboid(SPHAlgorithms::Point3F(0.f, 0.f, -1.f), 10.f, 10.f, 10.f));

    Collision::detectCollisions(particleVector, volume);

    EXPECT_FLOAT_EQ(0.1f, particleVector[0].position.x);
    EXPECT_FLOAT_EQ(0.1f, particleVector[0].position.y);
    EXPECT_FLOAT_EQ(0.1f, particleVector[0].position.z);

    EXPECT_FLOAT_EQ(0.25f, particleVector[0].velocity.x);
    EXPECT_FLOAT_EQ(3.5f, particleVector[0].velocity.y);
    EXPECT_FLOAT_EQ(0.5f, particleVector[0].velocity.z);
}

void CollisionsTestSuite::twoOnBoundaryParticleCollision()
{
    ParticleVect particleVector = {Particle(SPHAlgorithms::Point3F(0.f, -0.5f, 1.f), 0.1f),
                                   Particle(SPHAlgorithms::Point3F(0.01f, -0.5f, 1.f), 0.1f)};
    particleVector[0].velocity = SPHAlgorithms::Point3F(1.f, -1.f, 1.f);
    particleVector[1].velocity = SPHAlgorithms::Point3F(-1.f, -1.f, 1.f);
    particleVector[0].neighbours = {1};
    particleVector[1].neighbours = {0};

    SPHAlgorithms::Volume volume(SPHAlgorithms::Cuboid(SPHAlgorithms::Point3F(0.f, 0.f, 0.f), 2.f, 2.f, 2.f));

    Collision::detectCollisions(particleVector, volume);

    EXPECT_FLOAT_EQ(0.5f, particleVector[0].velocity.x);
    EXPECT_FLOAT_EQ(0.5f, particleVector[0].velocity.y);
    EXPECT_FLOAT_EQ(1.f, particleVector[0].velocity.z);
    EXPECT_FLOAT_EQ(0.5f, particleVector[1].velocity.x);
    EXPECT_FLOAT_EQ(0.5f, particleVector[1].velocity.y);
    EXPECT_FLOAT_EQ(1.f, particleVector[1].velocity.z);
}

void CollisionsTestSuite::threeOnBoundaryParticleCollision()
{
    ParticleVect particleVector = {Particle(SPHAlgorithms::Point3F(-0.1f, 0.f, 1.f), 0.1f),
                                   Particle(SPHAlgorithms::Point3F(-0.1f, 0.2f, 1.f), 0.1f),
                                   Particle(SPHAlgorithms::Point3F(-0.1f, 0.4f, 1.f), 0.1f)};
    particleVector[0].velocity = SPHAlgorithms::Point3F(-1.f, 0.f, 1.f);
    particleVector[1].velocity = SPHAlgorithms::Point3F(-1.f, 0.f, 1.f);
    particleVector[2].velocity = SPHAlgorithms::Point3F(-1.f, 0.f, 1.f);

    SPHAlgorithms::Volume volume(SPHAlgorithms::Cuboid(SPHAlgorithms::Point3F(0.f, 0.f, 0.f), 2.f, 2.f, 2.f));

    Collision::detectCollisions(particleVector, volume);

    EXPECT_FLOAT_EQ(0.5f, particleVector[0].velocity.x);
    EXPECT_FLOAT_EQ(0.f, particleVector[0].velocity.y);
    EXPECT_FLOAT_EQ(1.f, particleVector[0].velocity.z);
    EXPECT_FLOAT_EQ(0.5f, particleVector[1].velocity.x);
    EXPECT_FLOAT_EQ(0.f, particleVector[1].velocity.y);
    EXPECT_FLOAT_EQ(1.f, particleVector[1].velocity.z);
    EXPECT_FLOAT_EQ(0.5f, particleVector[2].velocity.x);
    EXPECT_FLOAT_EQ(0.f, particleVector[2].velocity.y);
    EXPECT_FLOAT_EQ(1.f, particleVector[2].velocity.z);
}

} // namespace TestEnvironment
} // namespace SPHSDK

using namespace SPHSDK::TestEnvironment;

TEST(CollisionsTestSuite, twoParticleCollision)
{
    CollisionsTestSuite::twoParticleCollision();
}

TEST(CollisionsTestSuite, threeParticleCollision)
{
    CollisionsTestSuite::threeParticleCollision();
}

TEST(CollisionsTestSuite, fourParticleCollision)
{
    CollisionsTestSuite::fourParticleCollision();
}

TEST(CollisionsTestSuite, twoAndTwoParticleCollision)
{
    CollisionsTestSuite::twoAndTwoParticleCollision();
}

TEST(CollisionsTestSuite, oneAndFiveParticleCollision)
{
    CollisionsTestSuite::oneAndFiveParticleCollision();
}

TEST(CollisionsTestSuite, oneAndFourParticleCollision)
{
    CollisionsTestSuite::oneAndFourParticleCollision();
}

TEST(CollisionsTestSuite, oneAndEightParticleCollision)
{
    CollisionsTestSuite::oneAndEightParticleCollision();
}

TEST(CollisionsTestSuite, oneOnBoundaryParticleCollision)
{
    CollisionsTestSuite::oneOnBoundaryParticleCollision();
}

TEST(CollisionsTestSuite, twoOnBoundaryParticleCollision)
{
    CollisionsTestSuite::twoOnBoundaryParticleCollision();
}

TEST(CollisionsTestSuite, threeOnBoundaryParticleCollision)
{
    CollisionsTestSuite::threeOnBoundaryParticleCollision();
}
