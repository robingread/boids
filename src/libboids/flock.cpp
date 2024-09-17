#include "flock.h"
#include "utils.h"
#include <iostream>

namespace boids {

Flock::Flock() {
    m_idCount = 0;
    m_boidMap = {};

    m_config.neighbourhoodRadius = 50.0f;
    m_config.alignmentScale      = 0.1f;
    m_config.coheasionScale      = 0.05f;
    m_config.repelScale          = 0.075f;
    m_config.maxVelocity         = 2.0f;

    m_predatorCfg.neighbourhoodRadius = 120.0f;
    m_predatorCfg.alignmentScale      = 1.0f;
    m_predatorCfg.coheasionScale      = 1.0f;
    m_predatorCfg.repelScale          = 0.05f;
    m_predatorCfg.maxVelocity         = 1.75f;
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
    std::vector<Boid>&       boids     = m_boidMap[BoidType::BOID];
    const std::vector<Boid>& obstacles = m_boidMap[BoidType::OBSTACLE];
    for (Boid& b : boids) {
        const std::vector<boids::Boid> neighbours = boids::utils::getBoidNeighbourhood(
            b, boids, m_config.neighbourhoodRadius, m_sceneBounds);

        const std::vector<boids::Boid> obstacleNeighbours = boids::utils::getBoidNeighbourhood(
            b, obstacles, m_config.neighbourhoodRadius, m_sceneBounds);

        const QVector2D alignVector =
            boids::utils::calculateAlignmentVector(b, neighbours).normalized() *
            m_config.alignmentScale;

        const QVector2D cohesionVector =
            boids::utils::calculateCohesionVector(b, neighbours).normalized() *
            m_config.coheasionScale;

        const QVector2D repelVec =
            boids::utils::calculateSeparationVector(b, neighbours, 75.0f).normalized() *
            m_config.repelScale;

        const QVector2D obstacleVec =
            boids::utils::calculateSeparationVector(b, obstacleNeighbours, 150.0f).normalized() *
            m_config.repelScale * 2.0f;

        const QVector2D noiseVec = boids::utils::generateRandomVelocityVector(0.0001f);

        const QPointF& p = b.getPosition();

        QVector2D v =
            b.getVelocity() + alignVector + cohesionVector + repelVec + obstacleVec + noiseVec;

        boids::utils::clipVectorMangitude(v, m_config.maxVelocity);

        b.setPosition(QPointF(p.x() + v.x(), p.y() + v.y()));
        utils::wrapBoidPosition(b, m_sceneBounds);

        b.setVelocity(v);
    }
}

}; // namespace boids
