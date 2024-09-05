#ifndef FLOCK_H
#define FLOCK_H

#include "boids.h"
#include <QRectF>

namespace boids {

class Flock
{
public:
    Flock();

    int addBoid(const float x, const float y);
    void clearBoids();
    int getNumBoids() const;
    std::vector<Boid> getBoids() const;

    QRectF getSceneBounds() const;
    void setSceneBounds(const QRectF &bounds);

    void update();

private:
    std::size_t m_idCount;
    std::vector<Boid> m_boids;
    QRectF m_sceneBounds;
};

};

#endif // FLOCK_H
