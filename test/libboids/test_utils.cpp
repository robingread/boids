#include <boids.h>
#include <catch2/catch.hpp>
#include <gtest/gtest.h>
#include <utils.h>

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

struct BoidDistanceData {
    QRectF      bounds;
    boids::Boid b1;
    boids::Boid b2;
    float       expected;
};

class DistanceBetweenBoidsTest : public ::testing::TestWithParam<BoidDistanceData> {};

TEST_P(DistanceBetweenBoidsTest, test) {
    const auto  params = GetParam();
    const float res    = boids::utils::distanceBetweenBoids(params.b1, params.b2, params.bounds);
    ASSERT_FLOAT_EQ(params.expected, res);
}

INSTANTIATE_TEST_SUITE_P(
    utils, DistanceBetweenBoidsTest,
    ::testing::Values(BoidDistanceData{QRectF(0.0f, 0.0f, 20.0f, 10.0f),
                                       boids::Boid(0, 19.0f, 1.0f), boids::Boid(1, 1.0f, 1.0f),
                                       2.0f},
                      BoidDistanceData{QRectF(0.0f, 0.0f, 20.0f, 10.0f), boids::Boid(0, 1.0f, 2.0f),
                                       boids::Boid(1, 1.0f, 9.0f), 3.0f},
                      BoidDistanceData{QRectF(0.0f, 0.0f, 20.0f, 10.0f), boids::Boid(0, 1.0f, 4.0f),
                                       boids::Boid(1, 1.0f, 5.0f), 1.0f}));

// Float Test Fixture
class GenerateRandomValueFloatTest : public ::testing::TestWithParam<std::tuple<float, float>> {};

TEST_P(GenerateRandomValueFloatTest, GeneratesValueInRange) {
    float a   = std::get<0>(GetParam());
    float b   = std::get<1>(GetParam());
    float res = boids::utils::generateRandomValue<float>(a, b);

    ASSERT_TRUE(res > a);
    ASSERT_TRUE(res <= b);
}

INSTANTIATE_TEST_SUITE_P(FloatRanges, GenerateRandomValueFloatTest,
                         ::testing::Values(std::make_tuple(0.0f, 1.0f),
                                           std::make_tuple(0.25f, 0.5f),
                                           std::make_tuple(-1.5f, -1.0f)));

// Int Test Fixture
class GenerateRandomValueIntTest
    : public ::testing::TestWithParam<std::tuple<int, int, std::size_t>> {};

TEST_P(GenerateRandomValueIntTest, GeneratesValueInRange) {
    int         a          = std::get<0>(GetParam());
    int         b          = std::get<1>(GetParam());
    std::size_t iterations = std::get<2>(GetParam());

    for (std::size_t i = 0; i < iterations; ++i) {
        int res = boids::utils::generateRandomValue<int>(a, b);
        ASSERT_GE(res, a);
        ASSERT_LE(res, b);
    }
}

INSTANTIATE_TEST_SUITE_P(IntRanges, GenerateRandomValueIntTest,
                         ::testing::Values(std::make_tuple(0, 10, 100),
                                           std::make_tuple(-10, 0, 100),
                                           std::make_tuple(0, 1, 100)));

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
    const QRectF      bounds(0.0f, 0.0f, 10.0f, 10.0f);
    const float       minDist = 1.5f;

    std::vector<boids::Boid> flock;
    flock.push_back(boids::Boid(1, 1.0f, 0.0f));
    flock.push_back(boids::Boid(2, 0.0f, 1.0f));
    flock.push_back(boids::Boid(3, 2.0f, 0.0f));

    std::vector<boids::Boid> neighbours =
        boids::utils::getBoidNeighbourhood(boid, flock, minDist, bounds);

    ASSERT_EQ(neighbours.size(), 2);
    ASSERT_EQ(neighbours.at(0).getId(), 1);
    ASSERT_EQ(neighbours.at(1).getId(), 2);
}

TEST(libboids_utils, getBoidNeighbourhood_2) {
    const boids::Boid boid(0, 0.0f, 0.0f);
    const QRectF      bounds(0.0f, 0.0f, 10.0f, 10.0f);
    const float       minDist = 1.5f;

    std::vector<boids::Boid> flock;
    flock.push_back(boid);
    flock.push_back(boids::Boid(1, 1.0f, 0.0f));
    flock.push_back(boids::Boid(2, 0.0f, 1.0f));
    flock.push_back(boids::Boid(3, 2.0f, 0.0f));

    std::vector<boids::Boid> neighbours =
        boids::utils::getBoidNeighbourhood(boid, flock, minDist, bounds);

    ASSERT_EQ(neighbours.size(), 2);
    ASSERT_EQ(neighbours.at(0).getId(), 1);
    ASSERT_EQ(neighbours.at(1).getId(), 2);
}

/**
 * @brief Test the boids::utils::getBoidNeighbourhood() method where some of the boids span across
 * the scene boundary. This method tests that the scene is wrapped correctly both vertically and
 * horizontally.
 *
 */
TEST(libboids_utils, getBoidNeighbourhood_checkSceneWrapping) {
    const boids::Boid boid0(0, 0.1f, 0.1f);
    const boids::Boid boid1(1, 0.9f, 0.1f);
    const boids::Boid boid2(2, 0.1f, 0.9f);
    const boids::Boid boid3(3, 0.9f, 0.9f);
    const boids::Boid boid4(4, 0.5f, 0.5f); // This boid should not be part of the neighbourhood.
    const QRectF      bounds(0.0f, 0.0f, 1.0f, 1.0f);
    const float       minDist = 0.4;

    std::vector<boids::Boid> flock;
    flock.push_back(boid0);
    flock.push_back(boid1);
    flock.push_back(boid2);
    flock.push_back(boid3);
    flock.push_back(boid4);

    std::vector<boids::Boid> neighbours =
        boids::utils::getBoidNeighbourhood(boid0, flock, minDist, bounds);

    ASSERT_EQ(neighbours.size(), 3);
    ASSERT_EQ(neighbours.at(0).getId(), 1);
    ASSERT_EQ(neighbours.at(1).getId(), 2);
    ASSERT_EQ(neighbours.at(2).getId(), 3);
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

/**
 * Test the scaleVector() method
 */
struct ScaleVectorData {
    QVector2D vector;
    float     scalar;
    QVector2D expected;
};

class ScaleVectorTest : public ::testing::TestWithParam<ScaleVectorData> {};

TEST_P(ScaleVectorTest, test) {
    const auto params = GetParam();
    const auto res    = boids::utils::scaleVector(params.vector, params.scalar);
    ASSERT_FLOAT_EQ(params.expected.x(), res.x());
    ASSERT_FLOAT_EQ(params.expected.y(), res.y());
}

INSTANTIATE_TEST_SUITE_P(
    utils, ScaleVectorTest,
    ::testing::Values(ScaleVectorData{QVector2D(3.0f, 4.0f), 5.0f, QVector2D(3.0f, 4.0f)},
                      ScaleVectorData{QVector2D(-3.0f, 0.0f), 10.0f, QVector2D(-10.0f, 0.0f)}));

/**
 * Test the shortestDistanceInWrappedSpace() method
 */
class DistanceTest
    : public ::testing::TestWithParam<std::tuple<float, float, float, float, float>> {};

TEST_P(DistanceTest, CalculatesShortestDistance) {
    const float x1  = std::get<0>(GetParam());
    const float x2  = std::get<1>(GetParam());
    const float min = std::get<2>(GetParam());
    const float max = std::get<3>(GetParam());
    const float exp = std::get<4>(GetParam());

    const float res = boids::utils::shortestDistanceInWrapedSpace(x1, x2, min, max);
    ASSERT_FLOAT_EQ(exp, res);
}

INSTANTIATE_TEST_SUITE_P(utils, DistanceTest,
                         ::testing::Values(std::make_tuple(-0.9f, 0.9f, -1.0f, 1.0f, -0.2f),
                                           std::make_tuple(0.7f, 0.9f, -1.0f, 1.0f, 0.2f),
                                           std::make_tuple(0.9f, 0.7f, -1.0f, 1.0f, -0.2f),
                                           std::make_tuple(-0.2f, 0.2f, -10.0f, 1.0f, 0.4f)));

/**
 * Test the wrapBoidPosition() method.
 */
struct WrapBoidPositionData {
    boids::Boid             boid;     // The boid to wrap.
    QRectF                  rect;     // The scene rectangle
    std::pair<float, float> expected; // Expected values in (x,y) format.
};

class WrapBoidPositionTest : public ::testing::TestWithParam<WrapBoidPositionData> {};

TEST_P(WrapBoidPositionTest, WrapTest) {
    auto params = GetParam();
    boids::utils::wrapBoidPosition(params.boid, params.rect);
    ASSERT_FLOAT_EQ(params.boid.getPosition().x(), params.expected.first);
    ASSERT_FLOAT_EQ(params.boid.getPosition().y(), params.expected.second);
}

INSTANTIATE_TEST_SUITE_P(utils, WrapBoidPositionTest,
                         ::testing::Values(WrapBoidPositionData{boids::Boid(0, 0.0f, 1.1f),
                                                                QRectF(0.0f, 0.0f, 1.0f, 1.0f),
                                                                std::make_pair(0.0f, 0.1f)},
                                           WrapBoidPositionData{boids::Boid(0, -0.1f, 0.0f),
                                                                QRectF(0.0f, 0.0f, 1.0f, 1.0f),
                                                                std::make_pair(0.9f, 0.0f)}));

/**
 * Test the wrapValue() method.
 */
struct WrapValueData {
    float value;
    float minValue;
    float maxValue;
    float expected;
};

class WrapValueTest : public ::testing::TestWithParam<WrapValueData> {};

TEST_P(WrapValueTest, test) {
    const auto  params = GetParam();
    const float res    = boids::utils::wrapValue(params.value, params.minValue, params.maxValue);
    ASSERT_FLOAT_EQ(res, params.expected);
}

INSTANTIATE_TEST_SUITE_P(utils, WrapValueTest,
                         ::testing::Values(WrapValueData{1.1f, -1.0f, 1.0f, -0.9f},
                                           WrapValueData{1.1f, 0.0f, 2.0f, 1.1f},
                                           WrapValueData{5.1f, 1.0f, 5.0f, 1.1f},
                                           WrapValueData{-0.1f, 0.0f, 1.0f, 0.9f},
                                           WrapValueData{0.0f, 0.0f, 1.0f, 0.0f}));

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

bool isApproxEqual(double a, double b, double epsilon = 1e-6) { return std::fabs(a - b) < epsilon; }

TEST_CASE("Test the calculateCohesionVector() method", "[utils]") {
    WHEN("There are no neighbours") {
        const boids::Boid boid(0, 0.0, 0.0);
        const auto        neighbours = std::vector<boids::Boid>();
        const QRectF      bounds(0.0, 0.0, 1.0, 1.0);
        const auto        result = boids::utils::calculateCohesionVector(boid, neighbours, bounds);

        THEN("The output vector should be zero") {
            REQUIRE(result.x() == 0.0);
            REQUIRE(result.y() == 0.0);
        }
    }
    WHEN("There is one neighbour Boid") {
        const boids::Boid boid(0, 0.0, 0.0);
        const auto        neighbours = std::vector<boids::Boid>({boids::Boid(1, 1.0, 0.0)});
        const QRectF      bounds(0.0, 0.0, 1.0, 1.0);

        THEN("The output vector should be zero") {
            const auto result = boids::utils::calculateCohesionVector(boid, neighbours, bounds);
            REQUIRE(result.x() >= 0.0);
            REQUIRE(result.y() >= 0.0);
        }
    }
    WHEN("There are two nighbours") {
        const QRectF      bounds(-1.0, -1.0, 2.0, 2.0);
        const boids::Boid boid(0, 0.0, 0.0);
        const auto        neighbours =
            std::vector<boids::Boid>({boids::Boid(1, 1.0, 0.0), boids::Boid(2, -1.0, 0.0)});

        THEN("The output vector shold be exactly zero") {
            const auto result = boids::utils::calculateCohesionVector(boid, neighbours, bounds);
            REQUIRE(result.x() == 0.0);
            REQUIRE(result.y() == 0.0);
        }
    }
    WHEN("There are three neighbours") {
        const QRectF      bounds(-4.0, -4.0, 8.0, 8.0);
        const boids::Boid boid(0, 0.0, 0.0);
        const auto        neighbours = std::vector<boids::Boid>(
            {boids::Boid(1, 1.0, 1.0), boids::Boid(2, 2.0, 1.0), boids::Boid(3, 3.0, 1.0)});

        const auto result = boids::utils::calculateCohesionVector(boid, neighbours, bounds);

        THEN("The output vector x should be within set bounds") {
            REQUIRE(result.x() >= 0.0);
            REQUIRE(result.x() <= 3.0);
            REQUIRE(result.y() >= 0.0);
            REQUIRE(result.y() <= 1.0);
        }
    }
    WHEN("There is one neighbour, wrapped around the X axis") {
        const QRectF      bounds(0.0, 0.0, 1.0, 1.0);
        const boids::Boid boid(0, 0.3, 0.0);
        const auto        neighbours = std::vector<boids::Boid>({boids::Boid(1.0, 0.95, 0.0)});
        const auto        result = boids::utils::calculateCohesionVector(boid, neighbours, bounds);

        THEN("The vector should point backwards in the X axis") {
            REQUIRE(result.x() < boid.getPosition().x());
        }
        THEN("The vector should be zero in the Y axis") { REQUIRE(result.y() == 0.0); }
    }
    WHEN("There is one neighbour, wrapped around the Y axis") {
        const QRectF      bounds(0.0, 0.0, 1.0, 1.0);
        const boids::Boid boid(0, 0.1, 0.3);
        const auto        neighbours = std::vector<boids::Boid>({boids::Boid(1.0, 0.1, 0.95)});
        const auto        result = boids::utils::calculateCohesionVector(boid, neighbours, bounds);

        THEN("The vector should be zero in the X axis") { REQUIRE(result.x() == 0.0); }
        THEN("The vector should be negative in the Y axis") {
            REQUIRE(result.y() < boid.getPosition().y());
        }
    }
    WHEN("There is one neighbour wrapped in both the X and Y axis") {
        const QRectF      bounds(0.0, 0.0, 1.0, 1.0);
        const boids::Boid boid(0, 0.3, 0.3);
        const auto        neighbours = std::vector<boids::Boid>({boids::Boid(1.0, 0.95, 0.95)});
        const auto        result = boids::utils::calculateCohesionVector(boid, neighbours, bounds);

        THEN("The vector should be negative in the X axis") {
            REQUIRE(result.x() < boid.getPosition().x());
        }
        THEN("The vector should be negative in the Y axis") {
            REQUIRE(result.y() < boid.getPosition().y());
        }
    }
}

TEST_CASE("Test the distanceVectorBetweenPoint() method", "[utils]") {
    const QRectF bounds(0.0, 0.0, 1.0, 1.0);

    auto [p1, p2, expected] =
        GENERATE(std::make_tuple(QPointF(0.0, 0.0), QPointF(0.0, 0.0), QVector2D(0.0, 0.0)),
                 std::make_tuple(QPointF(0.0, 0.0), QPointF(0.1, 0.1), QVector2D(0.1, 0.1)),
                 std::make_tuple(QPointF(0.1, 0.0), QPointF(0.9, 0.0), QVector2D(-0.2, 0.0)),
                 std::make_tuple(QPointF(0.0, 0.1), QPointF(0.0, 0.9), QVector2D(0.0, -0.2)),
                 std::make_tuple(QPointF(0.9, 0.0), QPointF(0.1, 0.0), QVector2D(0.2, 0.0)),
                 std::make_tuple(QPointF(0.0, 0.9), QPointF(0.0, 0.1), QVector2D(0.0, 0.2)));

    const auto result = boids::utils::distanceVectorBetweenPoints(p1, p2, bounds);

    REQUIRE(isApproxEqual(expected.x(), result.x()));
    REQUIRE(isApproxEqual(expected.y(), result.y()));
}