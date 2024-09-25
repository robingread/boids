#pragma once

#include "boids.h"
#include "config.h"
#include <QRectF>

namespace boids {

class Flock {
  public:
    Flock();

    int  addBoid(const float x, const float y, const BoidType type = BoidType::BOID);

    /**
     * @brief Clear all the boids in the flock.
     *
     * This includes all the normal boids, obstacles and predators.
     */
    void clearBoids();
    int  getNumBoids() const;
    // std::vector<Boid> getBoids() const;
    std::map<BoidType, std::vector<Boid>> getBoids() const;
    Config                                getConfig() const;
    void                                  setConfig(const Config& config);

    QRectF getSceneBounds() const;
    void   setSceneBounds(const QRectF& bounds);

    void update();

  private:
    Config                                m_config;
    Config                                m_predatorCfg;
    std::size_t                           m_idCount;
    QRectF                                m_sceneBounds;
    std::map<BoidType, std::vector<Boid>> m_boidMap;
};

}; // namespace boids
