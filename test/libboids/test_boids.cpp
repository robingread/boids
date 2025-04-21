#include <boids.h>
#include <gtest/gtest.h>
#include <math.h>
#include <memory>

class BasicBoidInit : public ::testing::Test {
  protected:
    uint16_t                     m_id;
    std::unique_ptr<boids::Boid> m_boid;

    virtual void SetUp() {
        m_id   = 10;
        m_boid = std::make_unique<boids::Boid>(m_id);
    }
};

/**
 * @brief Test that ID of the Boid matches what is given to it.
 */
TEST_F(BasicBoidInit, test_getId) { ASSERT_EQ(m_id, m_boid->getId()); }

/**
 * @brief Test that the initial heading for a Boid construct with only an ID is 0.0.
 */
TEST_F(BasicBoidInit, test_getAngle) { ASSERT_EQ(0.0f, m_boid->getAngle()); }

/**
 * @brief Test that the initial position of the Boid is (0.0, 0.0).
 */
TEST_F(BasicBoidInit, test_getPosition) {
    const QPointF exp(0.0, 0.0);
    const QPointF res = m_boid->getPosition();
    ASSERT_EQ(exp, res);
}

/**
 * @brief Test that the getType() method works as expected when no type is offered up at
 * construction.
 */
TEST_F(BasicBoidInit, test_getType) {
    const boids::BoidType exp = boids::BoidType::BOID;
    const boids::BoidType res = m_boid->getType();
    ASSERT_EQ(exp, res);
}

/**
 * @brief Test that the initial velocity of the Boid is (0.0, 0.0).
 */
TEST_F(BasicBoidInit, test_getVelocity) {
    const QVector2D exp(0.0, 0.0);
    const QVector2D res = m_boid->getVelocity();
    ASSERT_EQ(exp, res);
}

/**
 * @brief Test that the initial colour of the Boid is not black.
 */
TEST_F(BasicBoidInit, TestDefaultColorNotBlack) {
    const QColor color(0.0, 0.0, 0, 0);
    ASSERT_FALSE(color == m_boid->getColor());
}

/**
 * @brief Test that the setColour() method works as expected.
 */
TEST_F(BasicBoidInit, test_setColor) {
    const QColor exp(1, 2, 3);
    m_boid->setColor(exp);
    const QColor res = m_boid->getColor();
    ASSERT_EQ(exp, res);
}

/**
 * @brief Test that the setVelocity() method works as expected.
 */
TEST_F(BasicBoidInit, test_setVelocity) {
    const QVector2D exp(10.0, 20.0);
    m_boid->setVelocity(exp);
    const QVector2D res = m_boid->getVelocity();
    ASSERT_EQ(exp, res);
}

/**
 * @brief Create an initialisation where the Boid is of a OBSTACLE type
 */
class InitObstacleBoid : public ::testing::Test {
  protected:
    uint16_t                     m_id;
    boids::BoidType              m_type;
    std::unique_ptr<boids::Boid> m_boid;

    virtual void SetUp() {
        m_id   = 1;
        m_type = boids::BoidType::OBSTACLE;
        m_boid = std::make_unique<boids::Boid>(m_id, m_type);
    }
};

/**
 * @brief Test that the getType() method works as expected when no type is offered up at
 * construction.
 */
TEST_F(InitObstacleBoid, test_getType) {
    const boids::BoidType res = m_boid->getType();
    ASSERT_EQ(m_type, res);
}

/**
 * @brief Create an initialisation where the Boid is of a PREDATOR type
 */
class InitPredatorBoid : public ::testing::Test {
  protected:
    uint16_t                     m_id;
    boids::BoidType              m_type;
    std::unique_ptr<boids::Boid> m_boid;

    virtual void SetUp() {
        m_id   = 1;
        m_type = boids::BoidType::PREDATOR;
        m_boid = std::make_unique<boids::Boid>(m_id, m_type);
    }
};

/**
 * @brief Test that the getType() method works as expected when no type is offered up at
 * construction.
 */
TEST_F(InitPredatorBoid, test_getType) {
    const boids::BoidType res = m_boid->getType();
    ASSERT_EQ(m_type, res);
}

/*
 * Test that a boid in a velocity vector of (1, 0) has a angle of 0 rad.
 */
TEST(libboids_boids, getAngle_1) {
    const boids::Boid b(0, 0.0f, 0.0f, 1.0f, 0.0f);
    const float       res = b.getAngle();
    const float       exp = 0.0f;
    ASSERT_NEAR(res, exp, 0.01f);
}

/*
 * Test that a boid in a velocity vector of (0, 1) has a angle of PI/2 rad.
 */
TEST(libboids_boids, getAngle_2) {
    const boids::Boid b(0, 0.0f, 0.0f, 0.0f, 1.0f);
    const float       res = b.getAngle();
    const float       exp = M_PI_2;
    ASSERT_NEAR(res, exp, 0.01f);
}

/*
 * Test that a boid in a velocity vector of (0, -1) has a angle of -PI/2 rad.
 */
TEST(libboids_boids, getAngle_3) {
    const boids::Boid b(0, 0.0f, 0.0f, 0.0f, -1.0f);
    const float       res = b.getAngle();
    const float       exp = -M_PI_2;
    ASSERT_NEAR(res, exp, 0.01f);
}
