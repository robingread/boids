#pragma once

#include "boids.h"
#include "config.h"
#include <QRectF>

namespace boids {

class Flock {
  public:
    Flock();

    int               addBoid(const float x, const float y);
    void              clearBoids();
    int               getNumBoids() const;
    std::vector<Boid> getBoids() const;

    QRectF getSceneBounds() const;
    void   setSceneBounds(const QRectF& bounds);

    void update();

  private:
    Config            m_config;
    std::size_t       m_idCount;
    std::vector<Boid> m_boids;
    QRectF            m_sceneBounds;
};

}; // namespace boids
