#include <boids.h>
#include <gtest/gtest.h>
#include <utils.h>

/**
 * @brief Test the cohesion velocity vector calculation for a boid with no neighbours.
 */
TEST(libboids_utils, calculateCohesionVector_0) {
    const boids::Boid              boid(0, 0.0, 0.0);
    const std::vector<boids::Boid> neighbours;

    const QVector2D exp(0.0, 0.0);
    const QVector2D res = boids::utils::calculateCohesionVector(boid, neighbours);

    ASSERT_EQ(exp, res);
}

/**
 * @brief Test the cohesion velocity vector calculation for a boid with only one neighbour.
 */
TEST(libboids_utils, calculateCohesionVector_1) {
    const boids::Boid        boid(0, 0.0, 0.0);
    std::vector<boids::Boid> neighbours;
    neighbours.push_back(boids::Boid(1, 1.0, 0.0));

    const QVector2D res = boids::utils::calculateCohesionVector(boid, neighbours);

    ASSERT_GE(res.x(), 0.0f);
    ASSERT_FLOAT_EQ(res.y(), 0.0f);
}

/**
 * @brief Test the cohesion velocity vector calculation for a boid with two neighbours.
 */
TEST(libboids_utils, calculateCohesionVector_2) {
    const boids::Boid        boid(0, 0.0, 0.0);
    std::vector<boids::Boid> neighbours;
    neighbours.push_back(boids::Boid(1, 1.0, 0.0));
    neighbours.push_back(boids::Boid(2, -1.0, 0.0));

    const QVector2D exp(0.0, 0.0);
    const QVector2D res = boids::utils::calculateCohesionVector(boid, neighbours);

    ASSERT_FLOAT_EQ(exp.x(), res.x());
    ASSERT_FLOAT_EQ(exp.y(), res.y());
}

/**
 * @brief Test the cohesion velocity vector calculation for a boid with three neighbours.
 */
TEST(libboids_utils, calculateCohesionVector_3) {
    const boids::Boid        boid(0, 0.0, 0.0);
    std::vector<boids::Boid> neighbours;
    neighbours.push_back(boids::Boid(1, 1.0, 1.0));
    neighbours.push_back(boids::Boid(2, 2.0, 1.0));
    neighbours.push_back(boids::Boid(3, 3.0, 1.0));

    const QVector2D res = boids::utils::calculateCohesionVector(boid, neighbours);

    ASSERT_GE(res.x(), 0.0f);
    ASSERT_GE(res.y(), 0.0f);

    ASSERT_LE(res.x(), 3.0f);
    ASSERT_LE(res.y(), 1.0f);
}

/**
 * @brief Test that when there is neighourhood of size zero, then the alignment
 * vector is also (0.0, 0.0).
 */
TEST(libboids_utils, calculateAlignmentVector_0) {
    const boids::Boid              boid(0, 0.0, 0.0);
    const std::vector<boids::Boid> neighbours;

    const QVector2D exp(0.0, 0.0);
    const QVector2D res = boids::utils::calculateAlignmentVector(boid, neighbours);

    ASSERT_EQ(exp, res);
}

/**
 * @brief Test that when there is neighourhood of size one, then the alignment
 * vector is not zero.
 */
TEST(libboids_utils, calculateAlignmentVector_1) {
    const boids::Boid        boid(0, 0.0, 0.0);
    std::vector<boids::Boid> neighbours;
    neighbours.push_back(boids::Boid(1, 1.0, 1.0, 1.0, 0.0));

    const QVector2D exp(1.0, 0.0);
    const QVector2D res = boids::utils::calculateAlignmentVector(boid, neighbours);

    ASSERT_GE(res.x(), 0.0f);
    ASSERT_GE(res.y(), 0.0f);

    ASSERT_LE(res.x(), 1.0f);
    ASSERT_LE(res.y(), 1.0f);
}

/**
 * @brief Test that when there is neighourhood of size two, then the alignment
 * vector is not zero.
 */
TEST(libboids_utils, calculateAlignmentVector_2) {
    const boids::Boid        boid(0, 0.0, 0.0);
    std::vector<boids::Boid> neighbours;
    neighbours.push_back(boids::Boid(1, 1.0, 1.0, 1.0, 1.0));
    neighbours.push_back(boids::Boid(2, 1.0, 1.0, -1.0, 1.0));

    const QVector2D exp(0.0, 1.0);
    const QVector2D res = boids::utils::calculateAlignmentVector(boid, neighbours);

    ASSERT_FLOAT_EQ(exp.x(), res.x());
    ASSERT_FLOAT_EQ(exp.y(), res.y());
}

/**
 * @brief Test that a boid with a single neightbour, outside of the min distance
 * has a force vector length that IS zero.
 */
TEST(libboids_utils, calculateSeparationVector_0) {
    const float                    minDist = 0.5f;
    const boids::Boid              boid(0, 0.0f, 0.0f);
    const std::vector<boids::Boid> neighbours;

    const QVector2D res = boids::utils::calculateSeparationVector(boid, neighbours, minDist);
    const QVector2D exp(0.0f, 0.0f);

    ASSERT_EQ(exp, res);
}

/*
 * Test that a boid with a single neightbour, outside of the min distance
 * has a force vector length that IS zero.
 */
TEST(libboids_utils, calculateSeparationVector_1) {
    const float              minDist = 0.5f;
    const boids::Boid        boid(0, 0.0f, 0.0f);
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
TEST(libboids_utils, calculateSeparationVector_2) {
    const float              minDist = 1.0f;
    const boids::Boid        boid(0, 1.0f, 0.0f);
    std::vector<boids::Boid> neighbours;
    neighbours.push_back(boids::Boid(1, 1.0f, 0.1f));

    const QVector2D res = boids::utils::calculateSeparationVector(boid, neighbours, minDist);

    ASSERT_TRUE(res.length() > 0.0f);
}

/*
 * Test that a boid with a single neightbour, at exactly the same point
 * has a force vector magnitude equal to 1.0.
 */
TEST(libboids_utils, calculateSeparationVector_3) {
    const float              minDist = 1.0f;
    const boids::Boid        boid(0, 0.0f, 0.0f);
    std::vector<boids::Boid> neighbours;
    neighbours.push_back(boids::Boid(1, 0.0f, 0.0f));

    const QVector2D res = boids::utils::calculateSeparationVector(boid, neighbours, minDist);

    // ASSERT_TRUE(res.length() <= 1.0f);
    // ASSERT_TRUE(res.length() >= 0.95f);

    ASSERT_LE(res.length(), 1.0f);
    ASSERT_GE(res.length(), 0.95f);
    std::cout << "Res length: " << res.length() << std::endl;
}

/*
 * Test that a boid with a neighbour either side results in a separation vector of
 * length zero because the forces from the neighbours cancel each other out.
 */
TEST(libboids_utils, calculateSeparationVector_4) {
    const float              minDist = 1.0f;
    const boids::Boid        boid(0, 0.0f, 0.0f);
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
TEST(libboids_utils, calculateSeparationVector_5) {
    const float              minDist = 2.0f;
    const boids::Boid        boid(0, 0.0f, 0.0f);
    std::vector<boids::Boid> neighbours;
    neighbours.push_back(boids::Boid(1, -0.5f, 0.0f));
    neighbours.push_back(boids::Boid(2, 0.0f, 0.5f));
    neighbours.push_back(boids::Boid(3, 0.0f, -1.0f));

    const QVector2D res = boids::utils::calculateSeparationVector(boid, neighbours, minDist);
    const QVector2D exp(0.5f / 3.0f, 0.0f);

    ASSERT_GE(res.x(), 0.0f);
    ASSERT_GE(res.y(), 0.0f);
}

/*
 * Test that a boid with four neighbours all around results in a separation vector of
 * length zero because the forces from the neighbours cancel each other out.
 */
TEST(libboids_utils, calculateSeparationVector_6) {
    const float              minDist = 2.0f;
    const boids::Boid        boid(0, 0.0f, 0.0f);
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

TEST(libboids_utils, distanceBetweenBoids_1) {
    const boids::Boid b1(0, 0.0, 0.0);
    const boids::Boid b2(1, 1.0, 0.0);

    const float res = boids::utils::distanceBetweenBoids(b1, b2);
    const float exp = 1.0f;

    ASSERT_FLOAT_EQ(exp, res);
}

TEST(libboids_utils, distanceBetweenBoids_2) {
    const boids::Boid b1(0, 1.0, 0.0);
    const boids::Boid b2(1, 1.0, -5.0);

    const float res = boids::utils::distanceBetweenBoids(b1, b2);
    const float exp = 5.0f;

    ASSERT_FLOAT_EQ(exp, res);
}

/**
 * @brief Test that the boids::utils::distanceBetweenBoids() method handles the situation
 * where the shortest distance between Boids is actually via the wrapping of the scene.
 */
TEST(libboids_utils, distanceBetweenBoids_Bounded_1) {
    const QRectF      bounds(0.0f, 0.0f, 20.0f, 10.0f);
    const boids::Boid b1(0, 19.0f, 1.0f);
    const boids::Boid b2(1, 1.0f, 1.0f);

    const float exp = 2.0f;
    const float res = boids::utils::distanceBetweenBoids(b1, b2, bounds);

    ASSERT_FLOAT_EQ(exp, res);
}

/**
 * @brief Test that the boids::utils::distanceBetweenBoids() method handles the situation
 * where the shortest distance between Boids is actually via the wrapping of the scene.
 */
TEST(libboids_utils, distanceBetweenBoids_Bounded_2) {
    const QRectF      bounds(0.0f, 0.0f, 20.0f, 10.0f);
    const boids::Boid b1(0, 1.0f, 2.0f);
    const boids::Boid b2(1, 1.0f, 9.0f);

    const float exp = 3.0f;
    const float res = boids::utils::distanceBetweenBoids(b1, b2, bounds);

    ASSERT_FLOAT_EQ(exp, res);
}

/**
 * @brief Test that the boids::utils::distanceBetweenBoids() method handles the case where
 * the euclidian distance is indeed the shortest distance between two Boids.
 */
TEST(libboids_utils, distanceBetweenBoids_Bounded_3) {
    const QRectF      bounds(0.0f, 0.0f, 20.0f, 10.0f);
    const boids::Boid b1(0, 1.0f, 4.0f);
    const boids::Boid b2(1, 1.0f, 5.0f);

    const float exp = 1.0f;
    const float res = boids::utils::distanceBetweenBoids(b1, b2, bounds);

    ASSERT_FLOAT_EQ(exp, res);
}

TEST(libboids_utils, generateRandomValue_float_1) {
    const float a   = 0.0f;
    const float b   = 1.0f;
    const float res = boids::utils::generateRandomValue<float>(a, b);
    ASSERT_TRUE(res > a);
    ASSERT_TRUE(res <= b);
}

TEST(libboids_utils, generateRandomValue_float_2) {
    const float a   = 0.25f;
    const float b   = 0.5f;
    const float res = boids::utils::generateRandomValue<float>(a, b);
    ASSERT_TRUE(res > a);
    ASSERT_TRUE(res <= b);
}

TEST(libboids_utils, generateRandomValue_float_3) {
    const float a   = -1.5f;
    const float b   = -1.0f;
    const float res = boids::utils::generateRandomValue<float>(a, b);
    ASSERT_TRUE(res > a);
    ASSERT_TRUE(res <= b);
}

TEST(libboids_utils, generateRandomValue_int_1) {
    const int a   = 0;
    const int b   = 10;
    const int res = boids::utils::generateRandomValue<int>(a, b);
    ASSERT_TRUE(res >= a);
    ASSERT_TRUE(res < b);
}

TEST(libboids_utils, generateRandomValue_int_2) {
    for (std::size_t i = 0; i < 100; ++i) {
        const int a   = -10;
        const int b   = 0;
        const int res = boids::utils::generateRandomValue<int>(a, b);
        ASSERT_TRUE(res >= a);
        ASSERT_TRUE(res <= b);
    }
}

TEST(libboids_utils, generateRandomValue_int_3) {
    const int a   = 0;
    const int b   = 1;
    const int res = boids::utils::generateRandomValue<int>(a, b);
    ASSERT_TRUE(res >= a);
    ASSERT_TRUE(res < b);
}

TEST(libboids_utils, generateRandomVelocityVector_1) {
    for (std::size_t i = 0; i < 100; ++i) {
        const float     maxVel = boids::utils::generateRandomValue<float>(1.0f, 10.0f);
        const QVector2D res    = boids::utils::generateRandomVelocityVector(maxVel);
        ASSERT_TRUE(res.length() >= 0.0f);
        ASSERT_TRUE(res.length() <= maxVel);
    }
}

TEST(libboids_utils, getBoidNeighbourhood_1) {
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

TEST(libboids_utils, getBoidNeighbourhood_2) {
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

/**
 * @brief Test the getTotalNumBoids() method in the boids::utils namespace to check that
 * it returns the correct number of Boids in the input std::map. In this test, the input
 * map is empty.
 */
TEST(libboids_utils, getTotalNumBoids_0) {
    std::map<boids::BoidType, std::vector<boids::Boid>> boids = {};
    const std::size_t                                   res = boids::utils::getTotalNumBoids(boids);
    const std::size_t                                   exp = 0;
    ASSERT_EQ(res, exp);
}

/**
 * @brief Test the getTotalNumBoids() method in the boids::utils namespace to check that
 * it returns the correct number of Boids in the input std::map.
 */
TEST(libboids_utils, getTotalNumBoids_6) {
    std::map<boids::BoidType, std::vector<boids::Boid>> boids;
    boids[boids::BoidType::BOID]     = {boids::Boid(1, 1.0f, 2.0f), boids::Boid(2, 1.0f, 2.0f)};
    boids[boids::BoidType::OBSTACLE] = {boids::Boid(3, 1.0f, 2.0f), boids::Boid(4, 1.0f, 2.0f)};
    boids[boids::BoidType::PREDATOR] = {boids::Boid(5, 1.0f, 2.0f), boids::Boid(6, 1.0f, 2.0f)};
    const std::size_t res            = boids::utils::getTotalNumBoids(boids);
    const std::size_t exp            = 6;
    ASSERT_EQ(res, exp);
}

TEST(libboids_utils, scaleVector_1) {
    const QVector2D vec(3.0f, 4.0f);
    const float     scalar = 5.0f;

    const QVector2D res = boids::utils::scaleVector(vec, scalar);
    const QVector2D exp(3.0f, 4.0f);

    ASSERT_FLOAT_EQ(exp.x(), res.x());
    ASSERT_FLOAT_EQ(exp.y(), res.y());
}

TEST(libboids_utils, scaleVector_2) {
    const QVector2D vec(-3.0f, 0.0f);
    const float     scalar = 10.0f;

    const QVector2D res = boids::utils::scaleVector(vec, scalar);
    const QVector2D exp(-10.0f, 0.0f);

    ASSERT_FLOAT_EQ(exp.x(), res.x());
    ASSERT_FLOAT_EQ(exp.y(), res.y());
}

TEST(libboids_utils, wrapBoidPosition_1) {
    boids::Boid  boid(0, 0.0f, 1.1f);
    const QRectF rect(0.0f, 0.0f, 1.0f, 1.0f);
    boids::utils::wrapBoidPosition(boid, rect);

    ASSERT_FLOAT_EQ(boid.getPosition().x(), 0.0f);
    ASSERT_FLOAT_EQ(boid.getPosition().y(), 0.1f);
}

TEST(libboids_utils, wrapBoidPosition_2) {
    boids::Boid  boid(0, -0.1f, 0.0f);
    const QRectF rect(0.0f, 0.0f, 1.0f, 1.0f);
    boids::utils::wrapBoidPosition(boid, rect);

    ASSERT_FLOAT_EQ(boid.getPosition().x(), 0.9f);
    ASSERT_FLOAT_EQ(boid.getPosition().y(), 0.0f);
}

TEST(libboids_utils, wrapValue_1) {
    const float value    = 1.1f;
    const float minValue = 0.0f;
    const float maxValue = 1.0f;
    const float res      = boids::utils::wrapValue(value, minValue, maxValue);
    const float exp      = 0.1f;

    ASSERT_FLOAT_EQ(res, exp);
}

TEST(libboids_utils, wrapValue_2) {
    const float value    = 1.1f;
    const float minValue = -1.0f;
    const float maxValue = 1.0f;
    const float res      = boids::utils::wrapValue(value, minValue, maxValue);
    const float exp      = -0.9f;

    ASSERT_FLOAT_EQ(res, exp);
}

TEST(libboids_utils, wrapValue_3) {
    const float value    = 1.1f;
    const float minValue = 0.0f;
    const float maxValue = 2.0f;
    const float res      = boids::utils::wrapValue(value, minValue, maxValue);
    const float exp      = 1.1f;

    ASSERT_FLOAT_EQ(res, exp);
}

TEST(libboids_utils, wrapValue_4) {
    const float value    = 5.1f;
    const float minValue = 1.0f;
    const float maxValue = 5.0f;
    const float res      = boids::utils::wrapValue(value, minValue, maxValue);
    const float exp      = 1.1f;

    ASSERT_FLOAT_EQ(res, exp);
}

TEST(libboids_utils, wrapValue_5) {
    const float value    = -0.1f;
    const float minValue = 0.0f;
    const float maxValue = 1.0f;
    const float res      = boids::utils::wrapValue(value, minValue, maxValue);
    const float exp      = 0.9f;

    ASSERT_FLOAT_EQ(res, exp);
}

TEST(libboids_utils, wrapValue_6) {
    const float value    = 0.0f;
    const float minValue = 0.0f;
    const float maxValue = 1.0f;
    const float res      = boids::utils::wrapValue(value, minValue, maxValue);
    const float exp      = 0.0f;

    ASSERT_FLOAT_EQ(res, exp);
}

/**
 * @brief Test that the clipVectorMagnitude() method correctly clips a vector
 * whose length is lower than the minimum value.
 */
TEST(libboids_utils, clipVectorMangitude_UnderMin) {
    const float     minMag = 0.2f;
    const float     maxMag = 5.0f;
    QVector2D       vec(-0.1f, 0.0f);
    const QVector2D exp(-0.2f, 0.0f);
    boids::utils::clipVectorMangitude(vec, minMag, maxMag);
    ASSERT_EQ(vec, exp);
}

/**
 * @brief Test that the clipVectorMagnitude() method correctly clips a vector
 * whose length is greater than the maximum value.
 */
TEST(libboids_utils, clipVectorMangitude_OverMax) {
    const float     minMag = 0.2f;
    const float     maxMag = 5.0f;
    const QVector2D exp(5.0f, 0.0f);
    QVector2D       vec(10.0f, 0.0f);
    boids::utils::clipVectorMangitude(vec, minMag, maxMag);
    ASSERT_EQ(vec, exp);
}

/**
 * @brief Test that the clipVectorMagnitude() method correctly clips a vector
 * whose length is less than the maximum value.
 */
TEST(libboids_utils, clipVectorMangitude_UnderMax) {
    const float     minMag = 0.2f;
    const float     maxMag = 50.0f;
    const QVector2D exp(10.0f, 0.0f);
    QVector2D       vec(10.0f, 0.0f);
    boids::utils::clipVectorMangitude(vec, minMag, maxMag);
    ASSERT_EQ(vec, exp);
}

/**
 * @brief Test that the clipVectorMagnitude() method throws an exception if the min and max
 * magnitude values are the wrong way around.
 */
TEST(libboids_utils, clipVectorMangitude_InvalidArgs) {
    const float minMag = 2.0f;
    const float maxMag = 1.0f;
    QVector2D   vec(10.0f, 0.0f);
    ASSERT_THROW(boids::utils::clipVectorMangitude(vec, minMag, maxMag), std::invalid_argument);
}