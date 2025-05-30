#pragma once

#include "boids.h"
#include "config.h"
#include <QRectF>

namespace boids {

class Flock {
  public:
    Flock();

    /**
     * @brief Add a boid of a given type to the Flock, at a given coordinate.
     * @param x X coordinate.
     * @param y Y coordinate.
     * @param type Type of boids, defaults to BoidType::BOID.
     * @return Unique ID for the new boid.
     */
    int addBoid(const float x, const float y, const BoidType type = BoidType::BOID);

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

    /**
     * @brief Get the total number of boids. This covers all types of boids.
     * @return Number of boids.
     */
    int getNumBoids() const;

    /**
     * @brief Get the std::map of boids
     * @return The std::map containing all the different types of boids.
     */
    std::map<BoidType, std::vector<Boid>> getBoids() const;

    /**
     * @brief Get the configuration for a given boid type.
     * @param type Type of Boids.
     * @return Configuration object.
     */
    Config getConfig(const BoidType& type = BoidType::BOID) const;

    /**
     * @brief Set the configuration object for a given Boid type.
     * @param cfg Configuration object.
     * @param type The type of boid.
     */
    void setConfig(const Config& cfg, const BoidType& type = BoidType::BOID);

    /**
     * @brief Get the scene bounds that the Boids adhere to.
     * @return The scene bounds rectangle.
     */
    QRectF getSceneBounds() const;

    /**
     * @brief Set the Scene Bounds object
     * @param bounds Scene bounds rectangle object.
     */
    void setSceneBounds(const QRectF& bounds);

    /**
     * @brief Update the boids with a single step. This will update the normal boids, as well as the
     * predators.
     * @throws Exception if a config has not been set for the BoidType::BOID or BoidType::PREDATOR.
     */
    void update();

  private:
    std::size_t                           idCount_;
    QRectF                                sceneBounds_;
    std::map<BoidType, std::vector<Boid>> boidMap_;
    std::map<BoidType, Config>            cfgMap_;
};

}; // namespace boids
