#pragma once

#include "boids.h"
#include <Eigen/Core>
#include <QRectF>
#include <QVector2D>
#include <random>
#include <vector>

namespace boids {
namespace utils {

/**
 * @brief Calculate the velocity vector that aligns a boid with the direction of it's neighbourhood.
 *
 * This vector will help "steer" the boid in the same direction as all those in the
 * near vicinity. The velocity vector of each obtained, normalised and then weighted
 * by the inverse of the distance to that neighbour. This is done for each neighbour,
 * with the final alignment vector being the total sum.
 *
 * @param boid Boid to calculate the alignment vector for.
 * @param neighbours Neighbourhood of Boids.
 * @return Vector aligning the boid with the neighbours.
 */
QVector2D calculateAlignmentVector(const Boid& boid, const std::vector<Boid>& neighbours);

/**
 * @brief Calculate the vector that pulls a Boid towards the center of the neighbourood.
 *
 * This vector is calculated by calculating the center point of the neighbourhood,
 * subtracting the boid position from this, normalising the vector and giving it a small magnitude
 * to gently pull the boids towards the center of the neighbourhood.
 *
 * @param boid Boid to check calculate the cohesion vector for.
 * @param neighbours Neighbourhood around the Boid
 * @param bounds Scene bounds of the wrapped space.
 * @return Vector towards the center of the neighbourhood.
 */
QVector2D calculateCohesionVector(const Boid& boid, const std::vector<Boid>& neighbours,
                                  const QRectF& bounds);

/**
 * @brief Calculate the new color of a boids given the neighbourhood.
 *
 * @param boid Boid to calculate the color for.
 * @param neighbours Neighbourhood around the Boid.
 * @return QColor New Boid color.
 */
QColor calculateBoidColor(const Boid& boid, const std::vector<Boid>& neighbours);
/**
 * @brief Calculate the vector that repels a given Boids from the other boids within
 * the neighbourhood to maintain a minimum distance between them.
 *
 * The separation vector is calculated by calculating the difference between the boid and each
 * neighbour, and weighing the difference based on the distance itself. The result is that the
 * separation force is greater with boids that are closer to each other.
 *
 * @param boid Boid to check calculate the cohesion vector for.
 * @param neighbours Neighbourhood around the Boid.
 * @param minDist Minimum distance to retain between Boids.
 * @param bounds Scene bounds of the wrapped space.
 * @return Repelling vector.
 */
QVector2D calculateSeparationVector(const Boid& boid, const std::vector<Boid>& neighbours,
                                    const float minDist, const QRectF& bounds);

/**
 * @brief Calculate the euclidean distance between two Boids.
 * @param b1 First boid.
 * @param b2 Second boid.
 * @return Euclidean distance.
 */
float distanceBetweenBoids(const Boid& b1, const Boid& b2);

/**
 * @brief Calculate the alternative Euclidean distance between two boids, which looks at the
 * distance across the scene bounds (as the scene is wrapped).
 * @param b1 First boid.
 * @param b2 Second boid.
 * @param bounds Bounds of the scene.
 * @return Euclidean distance.
 */
float distanceBetweenBoids(const Boid& b1, const Boid& b2, const QRectF& bounds);

/**
 * @brief Calculate the vector between two points.
 *
 * The returned vector will be from p1 -> p2 nad takes into account the wrapped space.
 *
 * @param p1 First point.
 * @param p2 Second point.
 * @param bounds Scene bounds.
 * @return Displacement vector.
 * @throws An std::invalid_argument if the min value is greater than the max value.
 */
QVector2D distanceVectorBetweenPoints(const QPointF& p1, const QPointF& bp2, const QRectF& bounds);

/**
 * @brief Get the vector between two points taking the bounds of the space into account.
 *
 * The returned vector will be from p1 -> p2 nad takes into account the wrapped space.
 *
 * @param p1 First vector
 * @param p2 Second vector
 * @param bounds Scene bounds
 * @return Vector between points.
 */
Eigen::Vector2d vectorBetweenPoints(const Eigen::Vector2d& p1, const Eigen::Vector2d& p2,
                                    const QRectF& bounds);

template <typename T> T generateRandomValue(const T minValue, const T maxValue) {
    if (minValue > maxValue) {
        throw std::invalid_argument("The min value is greater than the max value");
    }

    std::random_device               rd;
    std::mt19937                     gen(rd());
    std::uniform_real_distribution<> distr(minValue, maxValue);
    return distr(gen);
}

/**
 * @brief Generate a random 2D vector with a maximum allowed magnitude.
 * @param maxMagnitude Maximum allowed length/magnitude.
 * @return Generated vector.
 */
QVector2D generateRandomVelocityVector(const float maxMagnitude);

/**
 * @brief Get the Boids that are within the neighbourhood of a given Boid. This will
 * include checking for distance across scene wrappings and not purely the classical
 * Euclidean distance.
 * @param boid Boids to get the neighbourhood for.
 * @param flock Flock of all the boids.
 * @param dist Neighbourhood distance.
 * @param bounds Bounds of the scene.
 * @return Vector of Boids that form the Neighbourhood.
 */
std::vector<Boid> getBoidNeighbourhood(const Boid& boid, const std::vector<Boid>& flock,
                                       const float& dist, const QRectF& bounds);

/**
 * @brief Get the total number in a map of different types of Boids.
 * @param boids Standard Map containing vectors of different types of Boids.
 * @return Total number of Boids.
 */
std::size_t getTotalNumBoids(const std::map<BoidType, std::vector<Boid>>& boids);

/**
 * @brief Get a QVector scaled by a given value.
 * @param vec Vector to scale.
 * @param scalar Scalar value.
 * @return QVector2D scaled.
 */
QVector2D scaleVector(const QVector2D& vec, const float& scalar);

/**
 * @brief Calculate the shortest distance between two points/values in a 1D wrapped space.
 *
 * This value is not absolute, it is signed.
 *
 * @param v1 First value point.
 * @param v2 Second value point.
 * @param min Minimum value in the wrapped space/dimension.
 * @param max Maximum value in the wrapped space/dimension.
 * @return Shortest distance.
 */
float shortestDistanceInWrappedSpace(const float& v1, const float& v2, const float& min,
                                     const float& max);

/**
 * @brief Wrap the position of a boid within a defined simulation area.
 * @param boid Boid to process.
 * @param rect Rectangle representing the simulation area.
 */
void wrapBoidPosition(Boid& boid, const QRectF& rect);

float wrapValue(const float& value, const float& minValue, const float& maxValue);

/**
 * @brief Wrap a 2D vector within a defined boundary space.
 * @param vector The vector/position to wrap.
 * @param bounds The bounds of the space.
 * @return Wrapped vector/position.
 */
Eigen::Vector2d wrapVector2d(const Eigen::Vector2d& vector, const QRectF& bounds);

/**
 * @brief Clip the length of a vector so that it falls within an allowed minimum/maximum range. This
 * operation is performed in place on the input vector.
 * @param vec Vector to clip.
 * @param minMagnitude Minimum magnitude/length value of the vector.
 * @param maxMagnitude Maximum magnitude/length value of the vector.
 * @throws std::invalid_argument If the minimum value is greater than the maximum.
 */
void clipVectorMagnitude(QVector2D& vec, const float& minMagnitude, const float& maxMagnitude);

} // namespace utils
}; // namespace boids
