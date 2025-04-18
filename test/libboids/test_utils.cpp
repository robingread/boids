#include <boids.h>
#include <catch2/catch.hpp>
#include <gtest/gtest.h>
#include <utils.h>

TEST(libboids_utils, generateRandomVelocityVector_1) {
    for (std::size_t i = 0; i < 100; ++i) {
        const float     maxVel = boids::utils::generateRandomValue<float>(1.0f, 10.0f);
        const QVector2D res    = boids::utils::generateRandomVelocityVector(maxVel);
        ASSERT_TRUE(res.length() >= 0.0f);
        ASSERT_TRUE(res.length() <= maxVel);
    }
}

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

bool isApproxEqual(double a, double b, double epsilon = 1e-6) { return std::fabs(a - b) < epsilon; }

TEST_CASE("Test the calculateAlignmentVector() method", "[utils]") {
    WHEN("There are no neighbours") {
        const boids::Boid              boid(0, 0.0, 0.0);
        const std::vector<boids::Boid> neighbours;

        const QVector2D exp(0.0, 0.0);
        const QVector2D res = boids::utils::calculateAlignmentVector(boid, neighbours);

        THEN("The alignment vector should be zero") { REQUIRE(exp == res); }
    }
    WHEN("The neighbourhood is of size one") {
        const boids::Boid        boid(0, 0.0, 0.0);
        std::vector<boids::Boid> neighbours;
        neighbours.push_back(boids::Boid(1, 1.0, 1.0, 1.0, 0.0));

        const QVector2D res = boids::utils::calculateAlignmentVector(boid, neighbours);

        THEN("The resulting vector should be greater than 0.0") {
            REQUIRE(res.x() >= 0.0);
            REQUIRE(res.y() >= 0.0);
        }

        THEN("The resulting vector should be less than 1.0") {
            REQUIRE(res.x() <= 1.0f);
            REQUIRE(res.y() <= 1.0f);
        }
    }
    WHEN("The neighbourhood size is two") {
        const boids::Boid        boid(0, 0.0, 0.0);
        std::vector<boids::Boid> neighbours;
        neighbours.push_back(boids::Boid(1, 1.0, 1.0, 1.0, 1.0));
        neighbours.push_back(boids::Boid(2, 1.0, 1.0, -1.0, 1.0));

        const QVector2D exp(0.0, 1.0);
        const QVector2D res = boids::utils::calculateAlignmentVector(boid, neighbours);

        THEN("The alignment vector should be straight forward") { REQUIRE(exp == res); }
    }
}

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

        THEN("The output vector should be exactly zero") {
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

TEST_CASE("Test the calculateSeparationVector() method", "[utils]") {
    WHEN("A boid with a single neighbour outside the main radius") {
        const QRectF                   bounds(0.0, 0.0, 1.0, 1.0);
        const float                    minDist = 0.5f;
        const boids::Boid              boid(0, 0.0f, 0.0f);
        const std::vector<boids::Boid> neighbours;

        const QVector2D result =
            boids::utils::calculateSeparationVector(boid, neighbours, minDist, bounds);

        THEN("The result vector should be (0.0, 0.0)") {
            REQUIRE(result.x() == 0.0);
            REQUIRE(result.y() == 0.0);
        }
    }
    WHEN("A boid with a different single neighbour outside the main radius") {
        const QRectF             bounds(0.0, 0.0, 2.0, 2.0);
        const float              minDist = 0.5f;
        const boids::Boid        boid(0, 0.0f, 0.0f);
        std::vector<boids::Boid> neighbours;
        neighbours.push_back(boids::Boid(1, 1.0f, 0.0f));

        const QVector2D result =
            boids::utils::calculateSeparationVector(boid, neighbours, minDist, bounds);

        THEN("The result vector should be (0.0, 0.0)") {
            REQUIRE(result.x() == 0.0);
            REQUIRE(result.y() == 0.0);
        }
    }
    WHEN("A boid with a single neighbour within the minimum radius") {
        const QRectF             bounds(0.0, 0.0, 1.0, 1.0);
        const float              minDist = 0.5f;
        const boids::Boid        boid(0, 1.0f, 0.0f);
        std::vector<boids::Boid> neighbours = {boids::Boid(1, 1.0f, 0.1f)};

        const QVector2D result =
            boids::utils::calculateSeparationVector(boid, neighbours, minDist, bounds);

        THEN("The length of the vector should be greater than zero") {
            REQUIRE(result.length() > 0.0);
        }
    }
    WHEN("Test that a boid with a single neighbour, at exactly the same point has a force vector "
         "magnitude equal to 1.0.") {
        const QRectF             bounds(0.0, 0.0, 1.0, 1.0);
        const float              minDist = 1.0f;
        const boids::Boid        boid(0, 0.0f, 0.0f);
        std::vector<boids::Boid> neighbours = {boids::Boid(1, 0.0f, 0.0f)};

        const QVector2D result =
            boids::utils::calculateSeparationVector(boid, neighbours, minDist, bounds);

        THEN("The result vector should be equal to 1.0") { REQUIRE(result.length() == 1.0f); }
    }
    WHEN("Test that a boid with a neighbour either side results in a separation vector of length "
         "zero because the forces from the neighbours cancel each other out.") {
        const QRectF             bounds(0.0, 0.0, 1.0, 1.0);
        const float              minDist = 1.0f;
        const boids::Boid        boid(0, 0.0f, 0.0f);
        std::vector<boids::Boid> neighbours;
        neighbours.push_back(boids::Boid(1, 0.5f, 0.0f));
        neighbours.push_back(boids::Boid(2, -0.5f, 0.0f));

        const QVector2D result =
            boids::utils::calculateSeparationVector(boid, neighbours, minDist, bounds);

        THEN("The result vector should be nearly 0.0") {
            REQUIRE(result.length() <= 0.05f);
            REQUIRE(result.length() >= 0.0f);
        }
    }
    WHEN("Test that a boid with three neighbours, two either side and one behind results in a "
         "forward pointing separation vector.") {
        const QRectF             bounds(-3.0, -3.0, 6.0, 6.0);
        const float              minDist = 2.0f;
        const boids::Boid        boid(0, 0.0f, 0.0f);
        std::vector<boids::Boid> neighbours = {
            boids::Boid(1, -0.5f, 0.0f), boids::Boid(2, 0.0f, 0.5f), boids::Boid(3, 0.0f, -1.0f)};

        const QVector2D res =
            boids::utils::calculateSeparationVector(boid, neighbours, minDist, bounds);

        THEN("The resulting vector should be (0.0, 0.0)") {
            REQUIRE(res.x() >= 0.0f);
            REQUIRE(res.y() >= 0.0f);
        }
    }
    WHEN("Test that a boid with four neighbours all around results in a separation vector of "
         "length zero because the forces from the neighbours cancel each other out.") {
        const QRectF             bounds(-5.0, 5.0, 10.0, 10.0);
        const float              minDist = 2.0f;
        const boids::Boid        boid(0, 0.0f, 0.0f);
        std::vector<boids::Boid> neighbours = {
            boids::Boid(1, 1.0f, 0.0f), boids::Boid(2, 0.0f, 1.0f), boids::Boid(3, 0.0f, -1.0f),
            boids::Boid(4, -1.0f, 0.0f)};

        const QVector2D res =
            boids::utils::calculateSeparationVector(boid, neighbours, minDist, bounds);

        THEN("") {
            REQUIRE(isApproxEqual(res.x(), 0.0));
            REQUIRE(isApproxEqual(res.y(), 0.0));
        }
    }
}

TEST_CASE("Test the clipVectorMagnitude() method", "[utils]") {
    WHEN("The vector length is within the allowed bounds") {
        const float minMag = 0.2f;
        const float maxMag = 5.0f;
        QVector2D   vec(-0.1f, 0.0f);
        boids::utils::clipVectorMangitude(vec, minMag, maxMag);

        THEN("The clipped vector X element should be -0.2") { REQUIRE(vec.x() == -0.2f); }
        THEN("The clipped vector Y element should be 0.0") { REQUIRE(vec.y() == 0.0f); }
    }
    WHEN("The vector length is under the maximum value") {
        const float     minMag = 0.2f;
        const float     maxMag = 50.0f;
        const QVector2D exp(10.0f, 0.0f);
        QVector2D       vec(10.0f, 0.0f);
        boids::utils::clipVectorMangitude(vec, minMag, maxMag);

        THEN("The clipped vector X element should be 10.0") { REQUIRE(vec.x() == 10.0f); }
        THEN("The clipped vector Y element should be 0.0") { REQUIRE(vec.y() == 0.0f); }
    }
    WHEN("The vector length is over the maximum value") {
        const float minMag = 0.2f;
        const float maxMag = 5.0f;
        QVector2D   vec(10.0f, 0.0f);
        boids::utils::clipVectorMangitude(vec, minMag, maxMag);

        THEN("The clipped vector X element should be 5.0") { REQUIRE(vec.x() == 5.0f); }
        THEN("The clipped vector Y element should be 0.0") { REQUIRE(vec.y() == 0.0f); }
    }
    WHEN("Calling the method when the min/max arguments are mixed up") {
        const float minMag = 2.0f;
        const float maxMag = 1.0f;
        QVector2D   vec(10.0f, 0.0f);

        THEN("An exception should be thrown") {
            REQUIRE_THROWS(boids::utils::clipVectorMangitude(vec, minMag, maxMag));
        }
    }
}

TEST_CASE("Test the distanceBetweenBoids() method", "[utils]") {
    WHEN("One boid is in front of the other") {
        const boids::Boid b1(0, 0.0, 0.0);
        const boids::Boid b2(1, 1.0, 0.0);

        const float res = boids::utils::distanceBetweenBoids(b1, b2);

        THEN("The distance should be 1.0") { REQUIRE(res == 1.0f); }
    }
    WHEN("One is to the side of the other") {
        const boids::Boid b1(0, 1.0, 0.0);
        const boids::Boid b2(1, 1.0, -5.0);

        const float res = boids::utils::distanceBetweenBoids(b1, b2);

        THEN("The distance should be 5.0") { REQUIRE(res == 5.0f); }
    }
    GIVEN("Boids are wrapped around the space") {
        const QRectF bounds(0.0f, 0.0f, 20.0f, 10.0f);
        WHEN("The boids are wrapped in the X axis") {
            const boids::Boid b1(0, 19.0f, 1.0f);
            const boids::Boid b2(1, 1.0f, 1.0f);
            const float       dist = boids::utils::distanceBetweenBoids(b1, b2, bounds);
            THEN("The distance should be 2.0") { REQUIRE(dist == 2.0f); }
        }
        WHEN("The boids are wrapped in the Y axis") {
            const boids::Boid b1(0, 1.0f, 2.0f);
            const boids::Boid b2(1, 1.0f, 9.0f);
            const float       dist = boids::utils::distanceBetweenBoids(b1, b2, bounds);
            THEN("The distance should be 3.0") { REQUIRE(dist == 3.0f); }
        }
        WHEN("The boids are not wrapped") {
            const boids::Boid b1(0, 1.0f, 4.0f);
            const boids::Boid b2(1, 1.0f, 5.0f);
            const float       dist = boids::utils::distanceBetweenBoids(b1, b2, bounds);
            THEN("The distance should be 1.0") { REQUIRE(dist == 1.0f); }
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

TEST_CASE("Test the generateRandomValue() method", "[utils]") {
    GIVEN("The use of the float type") {
        WHEN("The min/max are 0.0 and 1.0") {
            const float min   = 0.0f;
            const float max   = 1.0f;
            const float value = boids::utils::generateRandomValue<float>(min, max);
            THEN("The value should be above the min") { REQUIRE(value > min); }
            THEN("The value should be below the max") { REQUIRE(value <= max); }
        }
        WHEN("The min/max are 0.25 and 0.5") {
            const float min   = 0.25f;
            const float max   = 0.5f;
            const float value = boids::utils::generateRandomValue<float>(min, max);
            THEN("The value should be above the min") { REQUIRE(value > min); }
            THEN("The value should be below the max") { REQUIRE(value <= max); }
        }
        WHEN("The min/max are -1.5 and -1.0") {
            const float min   = -1.5f;
            const float max   = -1.0f;
            const float value = boids::utils::generateRandomValue<float>(min, max);
            THEN("The value should be above the min") { REQUIRE(value > min); }
            THEN("The value should be below the max") { REQUIRE(value <= max); }
        }
        WHEN("When the min value is larger than the max") {
            const float min = 1.0f;
            const float max = 0.0f;
            THEN("There should be an exception thrown") {
                REQUIRE_THROWS(boids::utils::generateRandomValue<float>(min, max));
            }
        }
    }
    GIVEN("The use of the int type") {
        WHEN("The min/max are 0.0 and 1.0") {
            const int min   = 0;
            const int max   = 10;
            const int value = boids::utils::generateRandomValue<int>(min, max);
            THEN("The value should be above the min") { REQUIRE(value >= min); }
            THEN("The value should be below the max") { REQUIRE(value <= max); }
        }
        WHEN("The min/max are 0.25 and 0.5") {
            const int min   = -10;
            const int max   = 0;
            const int value = boids::utils::generateRandomValue<int>(min, max);
            THEN("The value should be above the min") { REQUIRE(value >= min); }
            THEN("The value should be below the max") { REQUIRE(value <= max); }
        }
        WHEN("The min/max are 0 and 1") {
            const int min   = 0;
            const int max   = 1;
            const int value = boids::utils::generateRandomValue<int>(min, max);
            THEN("The value should be above the min") { REQUIRE(value >= min); }
            THEN("The value should be below the max") { REQUIRE(value <= max); }
        }
        WHEN("When the min value is larger than the max") {
            const int min = 1;
            const int max = 0;
            THEN("There should be an exception thrown") {
                REQUIRE_THROWS(boids::utils::generateRandomValue<int>(min, max));
            }
        }
    }
}

TEST_CASE("Test the getBoidNeighbourhood() method", "[utils]") {
    GIVEN("A flock of Boids") {
        const QRectF bounds(0.0f, 0.0f, 10.0f, 10.0f);
        const float  minDist = 1.5f;

        const boids::Boid boid(0, 0.0f, 0.0f);

        std::vector<boids::Boid> flock;
        flock.push_back(boids::Boid(1, 1.0f, 0.0f));
        flock.push_back(boids::Boid(2, 0.0f, 1.0f));
        flock.push_back(boids::Boid(3, 2.0f, 0.0f));

        WHEN("Calling the getBoidNeighbourhood() method") {
            const std::vector<boids::Boid> neighbours =
                boids::utils::getBoidNeighbourhood(boid, flock, minDist, bounds);

            THEN("The vector size should be 2") { REQUIRE(neighbours.size() == 2); }
            THEN("The neighbour IDs should be as expected") {
                REQUIRE(neighbours.at(0).getId() == 1);
                REQUIRE(neighbours.at(1).getId() == 2);
            }
        }

        WHEN("Adding the boid itself, and calling the getBoidNeighbourhood() method") {
            flock.push_back(boid);
            const std::vector<boids::Boid> neighbours =
                boids::utils::getBoidNeighbourhood(boid, flock, minDist, bounds);

            THEN("The vector size should be 2") { REQUIRE(neighbours.size() == 2); }
            THEN("The neighbour IDs should be as expected") {
                REQUIRE(neighbours.at(0).getId() == 1);
                REQUIRE(neighbours.at(1).getId() == 2);
            }
        }
    }
    GIVEN("A flock of boids that are wrapped around the space") {
        const boids::Boid boid0(0, 0.1f, 0.1f);
        const boids::Boid boid1(1, 0.9f, 0.1f);
        const boids::Boid boid2(2, 0.1f, 0.9f);
        const boids::Boid boid3(3, 0.9f, 0.9f);
        const boids::Boid boid4(4, 0.5f,
                                0.5f); // This boid should not be part of the neighbourhood.
        const QRectF      bounds(0.0f, 0.0f, 1.0f, 1.0f);
        const float       minDist = 0.4;

        std::vector<boids::Boid> flock;
        flock.push_back(boid0);
        flock.push_back(boid1);
        flock.push_back(boid2);
        flock.push_back(boid3);
        flock.push_back(boid4);

        WHEN("Calling the getBoidNeighbourhood() method") {
            const std::vector<boids::Boid> neighbours =
                boids::utils::getBoidNeighbourhood(boid0, flock, minDist, bounds);

            THEN("The vector size should be 3") { REQUIRE(neighbours.size() == 3); }
            THEN("The neighbour IDs should be as expected") {
                REQUIRE(neighbours.at(0).getId() == 1);
                REQUIRE(neighbours.at(1).getId() == 2);
                REQUIRE(neighbours.at(2).getId() == 3);
            }
        }
    }
}

TEST_CASE("Test the getTotalNumBoids() method", "[utils]") {
    GIVEN("An empty vector") {
        std::map<boids::BoidType, std::vector<boids::Boid>> boids;
        WHEN("Calling the getTotalNumBoids() method") {
            const std::size_t res = boids::utils::getTotalNumBoids(boids);
            THEN("The number og boids should be 0") { REQUIRE(res == 0); }
        }
    }
    GIVEN("A Vector with a mix of Boid types") {
        std::map<boids::BoidType, std::vector<boids::Boid>> boids;
        boids[boids::BoidType::BOID]     = {boids::Boid(1, 1.0f, 2.0f), boids::Boid(2, 1.0f, 2.0f)};
        boids[boids::BoidType::OBSTACLE] = {boids::Boid(3, 1.0f, 2.0f), boids::Boid(4, 1.0f, 2.0f)};
        boids[boids::BoidType::PREDATOR] = {boids::Boid(5, 1.0f, 2.0f), boids::Boid(6, 1.0f, 2.0f)};
        WHEN("Calling the getTotalNumBoids() method") {
            const std::size_t res = boids::utils::getTotalNumBoids(boids);
            THEN("The number og boids should be 6") { REQUIRE(res == 6); }
        }
    }
}

TEST_CASE("Test the scaleVector() method", "[utils]") {
    GIVEN("A vector with positive values") {
        const QVector2D vec(3.0f, 4.0f);

        WHEN("Calling the scaleVector() method with a scalar of 5.0") {
            const float scalar = 5.0f;
            const auto  result = boids::utils::scaleVector(vec, scalar);

            THEN("The vector should be unchanged") {
                const float epsilon = 0.001f;
                REQUIRE(result.x() == Approx(3.0f).epsilon(epsilon));
                REQUIRE(result.y() == Approx(4.0f).epsilon(epsilon));
            }
        }
    }

    GIVEN("A vector with only a positive X value") {
        const QVector2D vec(2.0f, 0.0f);

        WHEN("Calling the scaleVector() method with a scalar of 1.0") {
            const float scalar = 1.0f;
            const auto  result = boids::utils::scaleVector(vec, scalar);

            THEN("The vector values should be (1.0, 0.0)") {
                const float epsilon = 0.001f;
                REQUIRE(result.x() == Approx(scalar).epsilon(epsilon));
                REQUIRE(result.y() == Approx(0.0f).epsilon(epsilon));
            }
        }
    }

    GIVEN("A vector with only a positive Y value") {
        const QVector2D vec(0.0f, 4.0f);

        WHEN("Calling the scaleVector() method with a scalar of 1.0") {
            const float scalar = 1.0f;
            const auto  result = boids::utils::scaleVector(vec, scalar);

            THEN("The vector values should be (0.0, 1.0)") {
                const float epsilon = 0.001f;
                REQUIRE(result.x() == Approx(0.0f).epsilon(epsilon));
                REQUIRE(result.y() == Approx(scalar).epsilon(epsilon));
            }
        }
    }

    GIVEN("A vector with negative values") {
        const QVector2D vec(-3.0f, -4.0f);

        WHEN("Calling the scaleVector() method with a scalar of 5.0") {
            const float scalar = 5.0f;
            const auto  result = boids::utils::scaleVector(vec, scalar);

            THEN("The vector values should be (-3.0, -4.0)") {
                const float epsilon = 0.001f;
                REQUIRE(result.x() == Approx(-3.0f).epsilon(epsilon));
                REQUIRE(result.y() == Approx(-4.0f).epsilon(epsilon));
            }
        }
    }
}

TEST_CASE("Test the shortestDistanceInWrappedSpace() method", "[utils]") {
    const float space_min = -1.0f;
    const float space_max = 1.0f;
    const float epsilon   = 0.001;

    WHEN("x1 = -0.9 and x2 = 0.9") {
        const float x1 = -0.9f;
        const float x2 = 0.9f;
        const float result =
            boids::utils::shortestDistanceInWrapedSpace(x1, x2, space_min, space_max);
        THEN("The distance should be -0.2") { REQUIRE(result == Approx(-0.2).epsilon(epsilon)); }
    }

    WHEN("x1 = 0.7 and x2 = 0.9") {
        const float x1 = 0.7f;
        const float x2 = 0.9f;
        const float result =
            boids::utils::shortestDistanceInWrapedSpace(x1, x2, space_min, space_max);
        THEN("The distance should be 0.2") { REQUIRE(result == Approx(0.2).epsilon(epsilon)); }
    }

    WHEN("x1 = 0.9 and x2 = 0.7") {
        const float x1 = 0.9f;
        const float x2 = 0.7f;
        const float result =
            boids::utils::shortestDistanceInWrapedSpace(x1, x2, space_min, space_max);
        THEN("The distance should be -0.2") { REQUIRE(result == Approx(-0.2).epsilon(epsilon)); }
    }

    WHEN("x1 = 0.4 and x2 = 0.7") {
        const float x1 = 0.4f;
        const float x2 = 0.7f;
        const float result =
            boids::utils::shortestDistanceInWrapedSpace(x1, x2, space_min, space_max);
        THEN("The distance should be 0.3") { REQUIRE(result == Approx(0.3).epsilon(epsilon)); }
    }
}
