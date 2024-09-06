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

int Flock::getNumBoids() const { return m_boids.size(); }

QRectF Flock::getSceneBounds() const { return m_sceneBounds; }

void Flock::setSceneBounds(const QRectF& bounds) { m_sceneBounds = bounds; }

void Flock::update() {
    for (Boid& b : m_boids) {
        const std::vector<boids::Boid> neighbours =
            boids::utils::getBoidNeighbourhood(b, m_boids, 100.0f, m_sceneBounds);

        const QVector2D alignVector =
            boids::utils::calculateAlignmentVector(b, neighbours).normalized();
        const QVector2D cohesionVector =
            boids::utils::calculateCohesionVector(b, neighbours).normalized();
        const QVector2D repelVec =
            boids::utils::calculateSeparationVector(b, neighbours, 75.0f).normalized();

        const QPointF& p = b.getPosition();

        QVector2D v = b.getVelocity() + (alignVector * 0.05f) + (cohesionVector * 0.01f) +
                      (repelVec * 0.01f) + boids::utils::generateRandomVelocityVector(0.0001f);

        boids::utils::clipVectorMangitude(v, 2.0f);

        b.setPosition(QPointF(p.x() + v.x(), p.y() + v.y()));
        utils::wrapBoidPosition(b, m_sceneBounds);

        b.setVelocity(v);
    }
}

}; // namespace boids
