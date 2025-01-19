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

        const QVector2D cohesionVector =
            boids::utils::calculateCohesionVector(b, neighbours, sceneBounds);

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
    idCount_ = 0;
    boidMap_.clear();
    cfgMap_.clear();

    cfgMap_[BoidType::BOID]     = Config();
    cfgMap_[BoidType::PREDATOR] = Config();

    cfgMap_[BoidType::BOID].neighbourhoodRadius = 80.0f;
    cfgMap_[BoidType::BOID].alignmentScale      = 0.1f;
    cfgMap_[BoidType::BOID].coheasionScale      = 0.075f;
    cfgMap_[BoidType::BOID].repelScale          = 0.1f;
    cfgMap_[BoidType::BOID].maxVelocity         = 2.0f;
    cfgMap_[BoidType::BOID].obstacleRepelScale  = 0.5f;
    cfgMap_[BoidType::BOID].predatorRepelScale  = 5.0f;

    cfgMap_[BoidType::PREDATOR].neighbourhoodRadius = 120.0f;
    cfgMap_[BoidType::PREDATOR].alignmentScale      = 1.0f;
    cfgMap_[BoidType::PREDATOR].coheasionScale      = 1.0f;
    cfgMap_[BoidType::PREDATOR].repelScale          = 0.0f;
    cfgMap_[BoidType::PREDATOR].maxVelocity         = 1.75f;
    cfgMap_[BoidType::PREDATOR].obstacleRepelScale  = 1.0f;
    cfgMap_[BoidType::PREDATOR].predatorRepelScale  = 5.0f;
}

int Flock::addBoid(const float x, const float y, const BoidType type) {
    if (!boidMap_.contains(type)) {
        boidMap_[type] = std::vector<Boid>();
    }
    boidMap_[type].push_back(Boid(idCount_, x, y, type));
    return idCount_++;
}

void Flock::clearBoids() { boidMap_.clear(); }

void Flock::clearBoids(const BoidType& type) {
    if (!boidMap_.contains(type))
        return;
    boidMap_.at(type).clear();
}

std::map<BoidType, std::vector<Boid>> Flock::getBoids() const { return boidMap_; }

Config Flock::getConfig(const BoidType& type) const { return cfgMap_.at(type); }

void Flock::setConfig(const Config& cfg, const BoidType& type) { cfgMap_[type] = cfg; }

int Flock::getNumBoids() const { return boids::utils::getTotalNumBoids(boidMap_); }

QRectF Flock::getSceneBounds() const { return sceneBounds_; }

void Flock::setSceneBounds(const QRectF& bounds) { sceneBounds_ = bounds; }

void Flock::update() {
    updateBoids(boidMap_[BoidType::BOID], boidMap_[BoidType::BOID], boidMap_[BoidType::PREDATOR],
                boidMap_[BoidType::OBSTACLE], cfgMap_[BoidType::BOID], sceneBounds_);

    updateBoids(boidMap_[BoidType::PREDATOR], boidMap_[BoidType::BOID],
                boidMap_[BoidType::PREDATOR], boidMap_[BoidType::OBSTACLE],
                cfgMap_[BoidType::PREDATOR], sceneBounds_);
}

}; // namespace boids
