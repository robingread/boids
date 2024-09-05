#include <gtest/gtest.h>
#include <boids.h>
#include <utils.h>

/*
 * Test the cohesion velocity vector calculation for a boid with only one neighbour.
 */
TEST(libboids_utils, calculateCohesionVector_1)
{
    const boids::Boid boid(0, 0.0, 0.0);
    std::vector<boids::Boid> neighbours;
    neighbours.push_back(boids::Boid(1, 1.0, 0.0));

    const QVector2D exp(1.0, 0.0);
    const QVector2D res = boids::utils::calculateCohesionVector(boid, neighbours);

    ASSERT_FLOAT_EQ(exp.x(), res.x());
    ASSERT_FLOAT_EQ(exp.y(), res.y());
}


/*
 * Test the cohesion velocity vector calculation for a boid with two neighbours.
 */
TEST(libboids_utils, calculateCohesionVector_2)
{
    const boids::Boid boid(0, 0.0, 0.0);
    std::vector<boids::Boid> neighbours;
    neighbours.push_back(boids::Boid(1, 1.0, 0.0));
    neighbours.push_back(boids::Boid(2, -1.0, 0.0));

    const QVector2D exp(0.0, 0.0);
    const QVector2D res = boids::utils::calculateCohesionVector(boid, neighbours);

    ASSERT_FLOAT_EQ(exp.x(), res.x());
    ASSERT_FLOAT_EQ(exp.y(), res.y());
}


/*
 * Test the cohesion velocity vector calculation for a boid with three neighbours.
 */
TEST(libboids_utils, calculateCohesionVector_3)
{
    const boids::Boid boid(0, 0.0, 0.0);
    std::vector<boids::Boid> neighbours;
    neighbours.push_back(boids::Boid(1, 1.0, 1.0));
    neighbours.push_back(boids::Boid(2, 2.0, 1.0));
    neighbours.push_back(boids::Boid(3, 3.0, 1.0));

    const QVector2D exp(2.0, 1.0);
    const QVector2D res = boids::utils::calculateCohesionVector(boid, neighbours);

    ASSERT_FLOAT_EQ(exp.x(), res.x());
    ASSERT_FLOAT_EQ(exp.y(), res.y());
}


TEST(libboids_utils, calculateAlignmentVector_1)
{
    const boids::Boid boid(0, 0.0, 0.0);
    std::vector<boids::Boid> neighbours;
    neighbours.push_back(boids::Boid(1, 1.0, 1.0, 1.0, 0.0));

    const QVector2D exp(1.0, 0.0);
    const QVector2D res = boids::utils::calculateAlignmentVector(boid, neighbours);

    ASSERT_FLOAT_EQ(exp.x(), res.x());
    ASSERT_FLOAT_EQ(exp.y(), res.y());
}


TEST(libboids_utils, calculateAlignmentVector_2)
{
    const boids::Boid boid(0, 0.0, 0.0);
    std::vector<boids::Boid> neighbours;
    neighbours.push_back(boids::Boid(1, 1.0, 1.0, 1.0, 1.0));
    neighbours.push_back(boids::Boid(2, 1.0, 1.0, -1.0, 1.0));

    const QVector2D exp(0.0, 1.0);
    const QVector2D res = boids::utils::calculateAlignmentVector(boid, neighbours);

    ASSERT_FLOAT_EQ(exp.x(), res.x());
    ASSERT_FLOAT_EQ(exp.y(), res.y());
}


/*
 * Test that a boid with a single neightbour, outside of the min distance
 * has a force vector length that IS zero.
 */
TEST(libboids_utils, calculateSeparationVector_1)
{
    const float minDist = 0.5f;
    const boids::Boid boid(0, 0.0f, 0.0f);
    std::vector<boids::Boid> neighbours;
    neighbours.push_back(boids::Boid(1, 1.0f, 0.0f));

    const QVector2D res = boids::utils::calculateSeparationVector(boid, neighbours, minDist);
    const QVector2D exp(0.0f, 0.0f);

    ASSERT_FLOAT_EQ(exp.x(), res.x());
    ASSERT_FLOAT_EQ(exp.y(), res.y());
}


/*
 * Test that a boid with a single neightbour, within of the min distance
 * has a force vector length that IS NOT zero.
 */
TEST(libboids_utils, calculateSeparationVector_2)
{
    const float minDist = 1.0f;
    const boids::Boid boid(0, 1.0f, 0.0f);
    std::vector<boids::Boid> neighbours;
    neighbours.push_back(boids::Boid(1, 1.0f, 0.1f));

    const QVector2D res = boids::utils::calculateSeparationVector(boid, neighbours, minDist);

    ASSERT_TRUE(res.length() > 0.0f);
}


/*
 * Test that a boid with a single neightbour, at exactly the same point
 * has a force vector magnitude equal to 1.0.
 */
TEST(libboids_utils, calculateSeparationVector_3)
{
    const float minDist = 1.0f;
    const boids::Boid boid(0, 0.0f, 0.0f);
    std::vector<boids::Boid> neighbours;
    neighbours.push_back(boids::Boid(1, 0.0f, 0.0f));

    const QVector2D res = boids::utils::calculateSeparationVector(boid, neighbours, minDist);

    ASSERT_TRUE(res.length() <= 1.0f);
    ASSERT_TRUE(res.length() >= 0.95f);
}


/*
 * Test that a boid with a neighbour either side results in a separation vector of
 * length zero because the forces from the neighbours cancel each other out.
 */
TEST(libboids_utils, calculateSeparationVector_4)
{
    const float minDist = 1.0f;
    const boids::Boid boid(0, 0.0f, 0.0f);
    std::vector<boids::Boid> neighbours;
    neighbours.push_back(boids::Boid(1, 0.5f, 0.0f));
    neighbours.push_back(boids::Boid(2, -0.5f, 0.0f));

    const QVector2D res = boids::utils::calculateSeparationVector(boid, neighbours, minDist);

    ASSERT_TRUE(res.length() < 0.05f);
    ASSERT_TRUE(res.length() >= 0.0f);
}


/*
 * Test that a boid with three neighbours, two either side and one behind
 * results in a forward pointing separation vector.
 */
TEST(libboids_utils, calculateSeparationVector_5)
{
    const float minDist = 2.0f;
    const boids::Boid boid(0, 0.0f, 0.0f);
    std::vector<boids::Boid> neighbours;
    neighbours.push_back(boids::Boid(1, -0.5f, 0.0f));
    neighbours.push_back(boids::Boid(2, 0.0f, 0.5f));
    neighbours.push_back(boids::Boid(3, 0.0f, -1.0f));

    const QVector2D res = boids::utils::calculateSeparationVector(boid, neighbours, minDist);
    const QVector2D exp(0.5f / 3.0f, 0.0f);

    EXPECT_NEAR(exp.x(), res.x(), 0.01f);
    EXPECT_NEAR(exp.y(), res.y(), 0.01f);
}


/*
 * Test that a boid with four neighbours all around results in a separation vector of
 * length zero because the forces from the neighbours cancel each other out.
 */
TEST(libboids_utils, calculateSeparationVector_6)
{
    const float minDist = 2.0f;
    const boids::Boid boid(0, 0.0f, 0.0f);
    std::vector<boids::Boid> neighbours;
    neighbours.push_back(boids::Boid(1, 1.0f, 0.0f));
    neighbours.push_back(boids::Boid(2, 0.0f, 1.0f));
    neighbours.push_back(boids::Boid(3, 0.0f, -1.0f));
    neighbours.push_back(boids::Boid(4, -1.0f, 0.0f));

    const QVector2D res = boids::utils::calculateSeparationVector(boid, neighbours, minDist);
    const QVector2D exp(0.0f, 0.0f);

    EXPECT_NEAR(exp.x(), res.x(), 0.01f);
    EXPECT_NEAR(exp.y(), res.y(), 0.01f);
}


/*
 * Test that a boid with a distance that is zero has a repulsion weighting of less or equal to 1.0.
 */
TEST(libboids_utils, calculateRepulsionVector_1)
{
    const float dist = 0.0f;
    const float minDist = 1.0f;

    const float res = boids::utils::calculateRepulsionWeight(dist, minDist);

    ASSERT_TRUE(res <= 1.0f);
}


/*
 * Test that a boid with a distance that is large and well beyond the minimum
 * distance has a resulsion weighting of 0.0.
 */
TEST(libboids_utils, calculateRepulsionVector_2)
{
    const float dist = 1000.0f;
    const float minDist = 1.0f;

    const float res = boids::utils::calculateRepulsionWeight(dist, minDist);
    const float exp = 0.0f;

    ASSERT_FLOAT_EQ(res, exp);
}


TEST(libboids_utils, distanceBetweenBoids_1)
{
    const boids::Boid b1(0, 0.0, 0.0);
    const boids::Boid b2(1, 1.0, 0.0);

    const float res = boids::utils::distanceBetweenBoids(b1, b2);
    const float exp = 1.0f;

    ASSERT_FLOAT_EQ(exp, res);
}


TEST(libboids_utils, distanceBetweenBoids_2)
{
    const boids::Boid b1(0, 1.0, 0.0);
    const boids::Boid b2(1, 1.0, -5.0);

    const float res = boids::utils::distanceBetweenBoids(b1, b2);
    const float exp = 5.0f;

    ASSERT_FLOAT_EQ(exp, res);
}


TEST(libboids_utils, generateRandomValue_float_1)
{
    const float a = 0.0f;
    const float b = 1.0f;
    const float res = boids::utils::generateRandomValue<float>(a, b);
    ASSERT_TRUE(res > a);
    ASSERT_TRUE(res <= b);
}


TEST(libboids_utils, generateRandomValue_float_2)
{
    const float a = 0.25f;
    const float b = 0.5f;
    const float res = boids::utils::generateRandomValue<float>(a, b);
    ASSERT_TRUE(res > a);
    ASSERT_TRUE(res <= b);
}


TEST(libboids_utils, generateRandomValue_float_3)
{
    const float a = -1.5f;
    const float b = -1.0f;
    const float res = boids::utils::generateRandomValue<float>(a, b);
    ASSERT_TRUE(res > a);
    ASSERT_TRUE(res <= b);
}


TEST(libboids_utils, generateRandomValue_int_1)
{
    const int a = 0;
    const int b = 10;
    const int res = boids::utils::generateRandomValue<int>(a, b);
    ASSERT_TRUE(res >= a);
    ASSERT_TRUE(res < b);
}


TEST(libboids_utils, generateRandomValue_int_2)
{
    for (std::size_t i=0; i<100; ++i)
    {
        const int a = -10;
        const int b = 0;
        const int res = boids::utils::generateRandomValue<int>(a, b);
        ASSERT_TRUE(res >= a);
        ASSERT_TRUE(res <= b);
    }
}


TEST(libboids_utils, generateRandomValue_int_3)
{
    const int a = 0;
    const int b = 1;
    const int res = boids::utils::generateRandomValue<int>(a, b);
    ASSERT_TRUE(res >= a);
    ASSERT_TRUE(res < b);
}


TEST(libboids_utils, generateRandomVelocityVector_1)
{
    for (std::size_t i=0; i<100; ++i)
    {
        const float maxVel = boids::utils::generateRandomValue<float>(1.0f, 10.0f);
        const QVector2D res = boids::utils::generateRandomVelocityVector(maxVel);
        ASSERT_TRUE(res.length() >= 0.0f);
        ASSERT_TRUE(res.length() <= maxVel);
    }
}


TEST(libboids_utils, getBoidNeighbourhood_1)
{
    const boids::Boid boid(0, 0.0f, 0.0f);

    std::vector<boids::Boid> flock;
    flock.push_back(boids::Boid(1, 1.0f, 0.0f));
    flock.push_back(boids::Boid(2, 0.0f, 1.0f));
    flock.push_back(boids::Boid(3, 2.0f, 0.0f));

    std::vector<boids::Boid> neighbours = boids::utils::getBoidNeighbourhood(boid, flock, 1.5f);

    ASSERT_EQ(neighbours.size(), 2);
    ASSERT_EQ(neighbours.at(0).getId(), 1);
    ASSERT_EQ(neighbours.at(1).getId(), 2);
}


TEST(libboids_utils, getBoidNeighbourhood_2)
{
    const boids::Boid boid(0, 0.0f, 0.0f);

    std::vector<boids::Boid> flock;
    flock.push_back(boid);
    flock.push_back(boids::Boid(1, 1.0f, 0.0f));
    flock.push_back(boids::Boid(2, 0.0f, 1.0f));
    flock.push_back(boids::Boid(3, 2.0f, 0.0f));

    std::vector<boids::Boid> neighbours = boids::utils::getBoidNeighbourhood(boid, flock, 1.5f);

    ASSERT_EQ(neighbours.size(), 2);
    ASSERT_EQ(neighbours.at(0).getId(), 1);
    ASSERT_EQ(neighbours.at(1).getId(), 2);
}


TEST(libboids_utils, scaleVector_1)
{
    const QVector2D vec(3.0f, 4.0f);
    const float scalar = 5.0f;

    const QVector2D res = boids::utils::scaleVector(vec, scalar);
    const QVector2D exp(3.0f, 4.0f);

    ASSERT_FLOAT_EQ(exp.x(), res.x());
    ASSERT_FLOAT_EQ(exp.y(), res.y());
}


TEST(libboids_utils, scaleVector_2)
{
    const QVector2D vec(-3.0f, 0.0f);
    const float scalar = 10.0f;

    const QVector2D res = boids::utils::scaleVector(vec, scalar);
    const QVector2D exp(-10.0f, 0.0f);

    ASSERT_FLOAT_EQ(exp.x(), res.x());
    ASSERT_FLOAT_EQ(exp.y(), res.y());
}


TEST(libboids_utils, wrapBoidPosition_1)
{
    boids::Boid boid(0, 0.0f, 1.1f);
    const QRectF rect(0.0f, 0.0f, 1.0f, 1.0f);
    boids::utils::wrapBoidPosition(boid, rect);

    ASSERT_FLOAT_EQ(boid.getPosition().x(), 0.0f);
    ASSERT_FLOAT_EQ(boid.getPosition().y(), 0.1f);
}


TEST(libboids_utils, wrapBoidPosition_2)
{
    boids::Boid boid(0, -0.1f, 0.0f);
    const QRectF rect(0.0f, 0.0f, 1.0f, 1.0f);
    boids::utils::wrapBoidPosition(boid, rect);

    ASSERT_FLOAT_EQ(boid.getPosition().x(), 0.9f);
    ASSERT_FLOAT_EQ(boid.getPosition().y(), 0.0f);
}


TEST(libboids_utils, wrapValue_1)
{
    const float value = 1.1f;
    const float minValue = 0.0f;
    const float maxValue = 1.0f;
    const float res = boids::utils::wrapValue(value, minValue, maxValue);
    const float exp = 0.1f;

    ASSERT_FLOAT_EQ(res, exp);
}


TEST(libboids_utils, wrapValue_2)
{
    const float value = 1.1f;
    const float minValue = -1.0f;
    const float maxValue = 1.0f;
    const float res = boids::utils::wrapValue(value, minValue, maxValue);
    const float exp = -0.9f;

    ASSERT_FLOAT_EQ(res, exp);
}


TEST(libboids_utils, wrapValue_3)
{
    const float value = 1.1f;
    const float minValue = 0.0f;
    const float maxValue = 2.0f;
    const float res = boids::utils::wrapValue(value, minValue, maxValue);
    const float exp = 1.1f;

    ASSERT_FLOAT_EQ(res, exp);
}


TEST(libboids_utils, wrapValue_4)
{
    const float value = 5.1f;
    const float minValue = 1.0f;
    const float maxValue = 5.0f;
    const float res = boids::utils::wrapValue(value, minValue, maxValue);
    const float exp = 1.1f;

    ASSERT_FLOAT_EQ(res, exp);
}


TEST(libboids_utils, wrapValue_5)
{
    const float value = -0.1f;
    const float minValue = 0.0f;
    const float maxValue = 1.0f;
    const float res = boids::utils::wrapValue(value, minValue, maxValue);
    const float exp = 0.9f;

    ASSERT_FLOAT_EQ(res, exp);
}


TEST(libboids_utils, wrapValue_6)
{
    const float value = 0.0f;
    const float minValue = 0.0f;
    const float maxValue = 1.0f;
    const float res = boids::utils::wrapValue(value, minValue, maxValue);
    const float exp = 0.0f;

    ASSERT_FLOAT_EQ(res, exp);
}