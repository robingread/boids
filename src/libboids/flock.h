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

    /**
     * @brief Clear all the boids of a given type.
     * @param type The enum for the type of Boids to clear.
     */
    void clearBoids(const BoidType& type);

    std::map<BoidType, std::vector<Boid>> getBoids() const;
    Config                                getConfig() const;
    void                                  setConfig(const Config& config);

    QRectF getSceneBounds() const;
    void   setSceneBounds(const QRectF& bounds);

    /**
     * @brief Update the boids with a single step. This will update the normal boids, as well as the
     * predators.
     * @throws Exception if a config has not been set for the BoidType::BOID or BoidType::PREDATOR.
     */
    void update();

  private:
    Config                                m_config;
    Config                                m_predatorCfg;
    std::size_t                           m_idCount;
    QRectF                                m_sceneBounds;
    std::map<BoidType, std::vector<Boid>> m_boidMap;
};

}; // namespace boids
