#include "flock.h"
#include "utils.h"
#include <iostream>

namespace boids {

Flock::Flock() { m_idCount = 0; }

int Flock::addBoid(const float x, const float y) {
    m_boids.push_back(Boid(m_idCount, x, y));
    return m_idCount++;
}

void Flock::clearBoids() { m_boids.clear(); }

std::vector<Boid> Flock::getBoids() const { return m_boids; }

Config Flock::getConfig() const { return m_config; }

void Flock::setConfig(const Config& config) { m_config = config; }

int Flock::getNumBoids() const { return m_boids.size(); }

QRectF Flock::getSceneBounds() const { return m_sceneBounds; }

void Flock::setSceneBounds(const QRectF& bounds) { m_sceneBounds = bounds; }

void Flock::update() {
    for (Boid& b : m_boids) {
        const std::vector<boids::Boid> neighbours = boids::utils::getBoidNeighbourhood(
            b, m_boids, m_config.neighbourhoodRadius, m_sceneBounds);

        const QVector2D alignVector =
            boids::utils::calculateAlignmentVector(b, neighbours).normalized() *
            m_config.alignmentScale;

        const QVector2D cohesionVector =
            boids::utils::calculateCohesionVector(b, neighbours).normalized() *
            m_config.coheasionScale;

        const QVector2D repelVec =
            boids::utils::calculateSeparationVector(b, neighbours, 75.0f).normalized() *
            m_config.repelScale;

        const QVector2D noiseVec = boids::utils::generateRandomVelocityVector(0.0001f);

        const QPointF& p = b.getPosition();

        QVector2D v = b.getVelocity() + alignVector + cohesionVector + repelVec + noiseVec;

        boids::utils::clipVectorMangitude(v, m_config.maxVelocity);

        b.setPosition(QPointF(p.x() + v.x(), p.y() + v.y()));
        utils::wrapBoidPosition(b, m_sceneBounds);

        b.setVelocity(v);
    }
}

}; // namespace boids
