#include "flock.h"
#include "utils.h"
#include <iostream>

namespace boids {

/**
 * @brief Update a vector of Boid instances, taking into account the flock of boids,
 * obstacles and the predators.
 * @param boids Vector of Boid instances to update.
 * @param flock Vector of standard Boids.
 * @param predators Vector of Predator Boids.
 * @param obstacles Vector of Obstacle Boids.
 * @param cfg Configuration parameters to use for the update.
 * @param sceneBounds Bounds of the Scene.
 */
void updateBoids(std::vector<Boid>& boids, const std::vector<Boid>& flock,
                 const std::vector<Boid>& predators, const std::vector<Boid>& obstacles,
                 const Config& cfg, const QRectF& sceneBounds) {
    for (Boid& b : boids) {
        const std::vector<boids::Boid> neighbours =
            boids::utils::getBoidNeighbourhood(b, flock, cfg.neighbourhoodRadius, sceneBounds);

        const std::vector<boids::Boid> obstacleNeighbours =
            boids::utils::getBoidNeighbourhood(b, obstacles, cfg.neighbourhoodRadius, sceneBounds);

        const std::vector<boids::Boid> predatorNeighbours = boids::utils::getBoidNeighbourhood(
            b, predators, cfg.neighbourhoodRadius * 2.0f, sceneBounds);

        const QVector2D alignVector =
            boids::utils::calculateAlignmentVector(b, neighbours).normalized() * cfg.alignmentScale;

        const QVector2D cohesionVector =
            boids::utils::calculateCohesionVector(b, neighbours).normalized() * cfg.coheasionScale;

        const QVector2D repelVec =
            boids::utils::calculateSeparationVector(b, neighbours, 50.0f).normalized() *
            cfg.repelScale;

        const QVector2D obstacleVec =
            boids::utils::calculateSeparationVector(b, obstacleNeighbours, 150.0f).normalized() *
            cfg.repelScale * 2.0f;

        const QVector2D predatorVec =
            boids::utils::calculateSeparationVector(b, predatorNeighbours, 150.0f).normalized() *
            cfg.repelScale * 5.0f;

        const QVector2D noiseVec = boids::utils::generateRandomVelocityVector(0.0001f);

        const QPointF& p = b.getPosition();

        QVector2D v = b.getVelocity() + alignVector + cohesionVector + repelVec + obstacleVec +
                      predatorVec + noiseVec;

        boids::utils::clipVectorMangitude(v, cfg.maxVelocity);

        b.setPosition(QPointF(p.x() + v.x(), p.y() + v.y()));
        utils::wrapBoidPosition(b, sceneBounds);

        b.setVelocity(v);
    }
};

Flock::Flock() {
    m_idCount = 0;
    m_boidMap = {};

    m_config.neighbourhoodRadius = 80.0f;
    m_config.alignmentScale      = 0.1f;
    m_config.coheasionScale      = 0.075f;
    m_config.repelScale          = 0.1f;
    m_config.maxVelocity         = 2.0f;
    m_config.obstacleRepelScale  = 0.5f;
    m_config.predatorRepelScale  = 5.0f;

    m_predatorCfg.neighbourhoodRadius = 120.0f;
    m_predatorCfg.alignmentScale      = 1.0f;
    m_predatorCfg.coheasionScale      = 1.0f;
    m_predatorCfg.repelScale          = 0.0f;
    m_predatorCfg.maxVelocity         = 1.75f;
    m_predatorCfg.obstacleRepelScale  = 1.0f;
    m_predatorCfg.predatorRepelScale  = 5.0f;
}

int Flock::addBoid(const float x, const float y, const BoidType type) {
    if (!m_boidMap.contains(type)) {
        m_boidMap[type] = std::vector<Boid>();
    }
    m_boidMap[type].push_back(Boid(m_idCount, x, y, type));
    return m_idCount++;
}

void Flock::clearBoids() { m_boidMap.clear(); }

std::map<BoidType, std::vector<Boid>> Flock::getBoids() const { return m_boidMap; }

Config Flock::getConfig() const { return m_config; }

void Flock::setConfig(const Config& config) { m_config = config; }

int Flock::getNumBoids() const { return boids::utils::getTotalNumBoids(m_boidMap); }

QRectF Flock::getSceneBounds() const { return m_sceneBounds; }

void Flock::setSceneBounds(const QRectF& bounds) { m_sceneBounds = bounds; }

void Flock::update() {
    updateBoids(m_boidMap[BoidType::BOID], m_boidMap[BoidType::BOID], m_boidMap[BoidType::PREDATOR],
                m_boidMap[BoidType::OBSTACLE], m_config, m_sceneBounds);

    updateBoids(m_boidMap[BoidType::PREDATOR], m_boidMap[BoidType::BOID],
                m_boidMap[BoidType::PREDATOR], m_boidMap[BoidType::OBSTACLE], m_predatorCfg,
                m_sceneBounds);
}

}; // namespace boids
