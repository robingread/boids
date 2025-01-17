#include "flock.h"
#include "utils.h"

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

        const std::vector<boids::Boid> predatorNeighbours =
            boids::utils::getBoidNeighbourhood(b, predators, cfg.neighbourhoodRadius, sceneBounds);

        const QVector2D alignVector = boids::utils::calculateAlignmentVector(b, neighbours);

        const QVector2D cohesionVector = boids::utils::calculateCohesionVector(b, neighbours);

        const QVector2D repelVec =
            boids::utils::calculateSeparationVector(b, neighbours, cfg.repelMinDist);

        const QVector2D obstacleVec =
            boids::utils::calculateSeparationVector(b, obstacleNeighbours, cfg.repelMinDist * 2.0f);

        const QVector2D predatorVec =
            boids::utils::calculateSeparationVector(b, predatorNeighbours, cfg.repelMinDist * 5.0f);

        const QVector2D noiseVec = boids::utils::generateRandomVelocityVector(0.05f);

        const QPointF& p = b.getPosition();

        QVector2D v = b.getVelocity();
        v += (alignVector * cfg.alignmentScale);
        v += (cohesionVector * cfg.coheasionScale);
        v += (repelVec * cfg.repelScale);
        v += (obstacleVec * cfg.obstacleRepelScale);
        v += (predatorVec * cfg.predatorRepelScale);
        v += (noiseVec * 1.0f);

        boids::utils::clipVectorMangitude(v, 0.1f, cfg.maxVelocity);

        b.setPosition(QPointF(p.x() + v.x(), p.y() + v.y()));
        utils::wrapBoidPosition(b, sceneBounds);

        b.setVelocity(v);

        const QColor& colour = boids::utils::calculateBoidColor(b, neighbours);
        b.setColor(colour);
    }
};

Flock::Flock() {
    m_idCount = 0;
    m_boidMap.clear();
    m_cfgMap.clear();

    m_cfgMap[BoidType::BOID]     = Config();
    m_cfgMap[BoidType::PREDATOR] = Config();

    m_cfgMap[BoidType::BOID].neighbourhoodRadius = 80.0f;
    m_cfgMap[BoidType::BOID].alignmentScale      = 0.1f;
    m_cfgMap[BoidType::BOID].coheasionScale      = 0.075f;
    m_cfgMap[BoidType::BOID].repelScale          = 0.1f;
    m_cfgMap[BoidType::BOID].maxVelocity         = 2.0f;
    m_cfgMap[BoidType::BOID].obstacleRepelScale  = 0.5f;
    m_cfgMap[BoidType::BOID].predatorRepelScale  = 5.0f;

    m_cfgMap[BoidType::PREDATOR].neighbourhoodRadius = 120.0f;
    m_cfgMap[BoidType::PREDATOR].alignmentScale      = 1.0f;
    m_cfgMap[BoidType::PREDATOR].coheasionScale      = 1.0f;
    m_cfgMap[BoidType::PREDATOR].repelScale          = 0.0f;
    m_cfgMap[BoidType::PREDATOR].maxVelocity         = 1.75f;
    m_cfgMap[BoidType::PREDATOR].obstacleRepelScale  = 1.0f;
    m_cfgMap[BoidType::PREDATOR].predatorRepelScale  = 5.0f;
}

int Flock::addBoid(const float x, const float y, const BoidType type) {
    if (!m_boidMap.contains(type)) {
        m_boidMap[type] = std::vector<Boid>();
    }
    m_boidMap[type].push_back(Boid(m_idCount, x, y, type));
    return m_idCount++;
}

void Flock::clearBoids() { m_boidMap.clear(); }

void Flock::clearBoids(const BoidType& type) {
    if (!m_boidMap.contains(type))
        return;
    m_boidMap.at(type).clear();
}

std::map<BoidType, std::vector<Boid>> Flock::getBoids() const { return m_boidMap; }

Config Flock::getConfig(const BoidType& type) const { return m_cfgMap.at(type); }

void Flock::setConfig(const Config& cfg, const BoidType& type) { m_cfgMap[type] = cfg; }

int Flock::getNumBoids() const { return boids::utils::getTotalNumBoids(m_boidMap); }

QRectF Flock::getSceneBounds() const { return m_sceneBounds; }

void Flock::setSceneBounds(const QRectF& bounds) { m_sceneBounds = bounds; }

void Flock::update() {
    updateBoids(m_boidMap[BoidType::BOID], m_boidMap[BoidType::BOID], m_boidMap[BoidType::PREDATOR],
                m_boidMap[BoidType::OBSTACLE], m_cfgMap[BoidType::BOID], m_sceneBounds);

    updateBoids(m_boidMap[BoidType::PREDATOR], m_boidMap[BoidType::BOID],
                m_boidMap[BoidType::PREDATOR], m_boidMap[BoidType::OBSTACLE],
                m_cfgMap[BoidType::PREDATOR], m_sceneBounds);
}

}; // namespace boids
