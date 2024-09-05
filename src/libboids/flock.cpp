#include "flock.h"
#include "utils.h"
#include <iostream>

namespace boids {

Flock::Flock()
{
    m_idCount = 0;
}


int Flock::addBoid(const float x, const float y)
{
    m_boids.push_back(Boid(m_idCount, x, y));
    return m_idCount++;
}


void Flock::clearBoids()
{
    m_boids.clear();
}


std::vector<Boid> Flock::getBoids() const
{
    return m_boids;
}


int Flock::getNumBoids() const
{
    return m_boids.size();
}


QRectF Flock::getSceneBounds() const
{
    return m_sceneBounds;
}


void Flock::setSceneBounds(const QRectF &bounds)
{
    m_sceneBounds = bounds;
}


void Flock::update()
{
    for (Boid &b : m_boids)
    {
        const QPointF &p = b.getPosition();
        const QVector2D &v = b.getVelocity();
        b.setPosition(QPointF(p.x() + v.x(), p.y() + v.y()));
        utils::wrapBoidPosition(b, m_sceneBounds);
    }
}

};
