#include <gtest/gtest.h>
#include <flock.h>



TEST(libboids_flock, addBoid_1)
{
    boids::Flock flock;
    flock.addBoid(0.0f, 0.0f);
    ASSERT_EQ(flock.getNumBoids(), 1);
}


TEST(libboids_flock, addBoid_2)
{
    boids::Flock flock;
    for (std::size_t i=0; i-100; ++i)
    {
        flock.addBoid(0.0f, 0.0f);
    }
    ASSERT_EQ(flock.getNumBoids(), 100);
}


TEST(libboids_flock, addBoid_3)
{
    boids::Flock flock;
    ASSERT_EQ(flock.addBoid(0.0f, 0.0f), 0);
    ASSERT_EQ(flock.addBoid(0.0f, 0.0f), 1);
    flock.clearBoids();
    ASSERT_EQ(flock.addBoid(0.0f, 0.0f), 2);
}


TEST(libboids_flock, clearBoids_1)
{
    boids::Flock flock;
    flock.addBoid(0.0f, 0.0f);
    flock.clearBoids();
    ASSERT_EQ(flock.getNumBoids(), 0);
}


TEST(libboids_flock, clearBoids_2)
{
    boids::Flock flock;
    for (std::size_t i=0; i<1000; ++i)
    {
        flock.addBoid(0.0f, 0.0f);
    }
    flock.clearBoids();
    ASSERT_EQ(flock.getNumBoids(), 0);
}


TEST(libboids_flock, setSceneBounds)
{
    boids::Flock flock;
    const QRectF exp(0.0f, 0.0f, 10.0f, 10.0f);
    flock.setSceneBounds(exp);
    const QRectF res = flock.getSceneBounds();

    ASSERT_FLOAT_EQ(res.bottom(), exp.bottom());
    ASSERT_FLOAT_EQ(res.top(), exp.top());
    ASSERT_FLOAT_EQ(res.left(), exp.left());
    ASSERT_FLOAT_EQ(res.right(), exp.right());
}

