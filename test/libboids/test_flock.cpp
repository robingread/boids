#include <flock.h>
#include <gtest/gtest.h>

TEST(libboids_flock, addBoid_1) {
    boids::Flock flock;
    flock.addBoid(0.0f, 0.0f);
    ASSERT_EQ(flock.getNumBoids(), 1);
}

TEST(libboids_flock, addBoid_2) {
    boids::Flock flock;
    for (std::size_t i = 0; i - 100; ++i) {
        flock.addBoid(0.0f, 0.0f);
    }
    ASSERT_EQ(flock.getNumBoids(), 100);
}

TEST(libboids_flock, addBoid_3) {
    boids::Flock flock;
    ASSERT_EQ(flock.addBoid(0.0f, 0.0f), 0);
    ASSERT_EQ(flock.addBoid(0.0f, 0.0f), 1);
    flock.clearBoids();
    ASSERT_EQ(flock.addBoid(0.0f, 0.0f), 2);
}

TEST(libboids_flock, clearBoids_1) {
    boids::Flock flock;
    flock.addBoid(0.0f, 0.0f);
    flock.clearBoids();
    ASSERT_EQ(flock.getNumBoids(), 0);
}

TEST(libboids_flock, clearBoids_2) {
    boids::Flock flock;
    for (std::size_t i = 0; i < 1000; ++i) {
        flock.addBoid(0.0f, 0.0f);
    }
    flock.clearBoids();
    ASSERT_EQ(flock.getNumBoids(), 0);
}

/**
 * @brief Test that the Flock::clearBoids() method removes all boids from the object.
 */
TEST(libboids_flock, clearBoids_allBoids) {
    std::vector<boids::BoidType> types = {boids::BOID, boids::OBSTACLE, boids::PREDATOR};
    int                          count = 0;
    boids::Flock                 flock;

    for (const auto type : types) {
        for (std::size_t i = 0; i < 1000; ++i) {
            flock.addBoid(0.0f, 0.0f, type);
            count++;
        }
    }

    ASSERT_EQ(flock.getNumBoids(), count);
    flock.clearBoids();
    ASSERT_EQ(flock.getNumBoids(), 0);
}

/**
 * @brief Test that calling the Flock::clearBoids() with a given type method without adding any
 * boids doesn't throw an exception.
 */
TEST(libboids_flock, clearBoids_boidType) {
    boids::Flock flock;
    ASSERT_NO_THROW(flock.clearBoids(boids::BOID));
}

/**
 * @brief Test the Flock::getBoids() method to check that it returns all the boids as expected.
 *
 */
TEST(libboids_flock, getBoids) {
    boids::Flock flock;

    for (std::size_t n = 0; n < 1; ++n) {
        flock.addBoid(0.0f, 0.0f, boids::BOID);
    }

    for (std::size_t n = 0; n < 2; ++n) {
        flock.addBoid(0.0f, 0.0f, boids::OBSTACLE);
    }

    for (std::size_t n = 0; n < 3; ++n) {
        flock.addBoid(0.0f, 0.0f, boids::PREDATOR);
    }

    const auto ret      = flock.getBoids();
    const auto retBoids = ret.at(boids::BOID);
    const auto retObs   = ret.at(boids::OBSTACLE);
    const auto retPred  = ret.at(boids::PREDATOR);

    ASSERT_EQ(retBoids.size(), 1);
    ASSERT_EQ(retObs.size(), 2);
    ASSERT_EQ(retPred.size(), 3);
}

TEST(libboids_flock, setSceneBounds) {
    boids::Flock flock;
    const QRectF exp(0.0f, 0.0f, 10.0f, 10.0f);
    flock.setSceneBounds(exp);
    const QRectF res = flock.getSceneBounds();

    ASSERT_FLOAT_EQ(res.bottom(), exp.bottom());
    ASSERT_FLOAT_EQ(res.top(), exp.top());
    ASSERT_FLOAT_EQ(res.left(), exp.left());
    ASSERT_FLOAT_EQ(res.right(), exp.right());
}

/**
 * @brief Test that getting and setting a new Config works.
 */
TEST(libboids_flock, getConfig) {
    boids::Flock  flock;
    boids::Config cfg;
    cfg.alignmentScale = 100.0f;

    ASSERT_NE(cfg.alignmentScale, flock.getConfig().alignmentScale);
    flock.setConfig(cfg);
    ASSERT_EQ(cfg.alignmentScale, flock.getConfig().alignmentScale);
}

/**
 * @brief Test Flock class that has instances of BOIDS, OBSTACLES and PREDATORS.
 */
class FullFlockTest : public testing::Test {

  protected:
    boids::Flock m_flock;

    void SetUp() {
        for (std::size_t i = 0; i < 10; ++i) {
            m_flock.addBoid(0.0f, 0.0f, boids::BOID);
        }
        for (std::size_t i = 0; i < 10; ++i) {
            m_flock.addBoid(0.0f, 0.0f, boids::OBSTACLE);
        }
        for (std::size_t i = 0; i < 10; ++i) {
            m_flock.addBoid(0.0f, 0.0f, boids::PREDATOR);
        }
    }
};

/**
 * @brief Test that the Flock::clearBoids() only clears the expected BOIDS boids.
 */
TEST_F(FullFlockTest, clearBoids_boids) {
    m_flock.clearBoids(boids::BOID);
    ASSERT_EQ(m_flock.getBoids().at(boids::BOID).size(), 0);
    ASSERT_EQ(m_flock.getBoids().at(boids::OBSTACLE).size(), 10);
    ASSERT_EQ(m_flock.getBoids().at(boids::PREDATOR).size(), 10);
}

/**
 * @brief Test that the Flock::clearBoids() only clears the expected OBSTACLE boids.
 */
TEST_F(FullFlockTest, clearBoids_obstalces) {
    m_flock.clearBoids(boids::OBSTACLE);
    ASSERT_EQ(m_flock.getBoids().at(boids::BOID).size(), 10);
    ASSERT_EQ(m_flock.getBoids().at(boids::OBSTACLE).size(), 0);
    ASSERT_EQ(m_flock.getBoids().at(boids::PREDATOR).size(), 10);
}

/**
 * @brief Test that the Flock::clearBoids() only clears the expected PREDATOR boids.
 */
TEST_F(FullFlockTest, clearBoids_predators) {
    m_flock.clearBoids(boids::PREDATOR);
    ASSERT_EQ(m_flock.getBoids().at(boids::BOID).size(), 10);
    ASSERT_EQ(m_flock.getBoids().at(boids::OBSTACLE).size(), 10);
    ASSERT_EQ(m_flock.getBoids().at(boids::PREDATOR).size(), 0);
}

/**
 * @brief Test that the Flock::update() doesn't throw any exceptions when called.
 */
TEST_F(FullFlockTest, update_noThrow) { ASSERT_NO_THROW(m_flock.update()); }