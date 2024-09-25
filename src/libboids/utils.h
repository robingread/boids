#pragma once

#include "boids.h"
#include <QRectF>
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
 * This vector is calculated by calculting the center point of the neighbourhood,
 * substracting the boid position from this, normalising the vector and giving it a small magnitude
 * to gently pull the boids towards the center of the neighbourhood.
 *
 * @param boid Boid to check calculate the cohesion vector for.
 * @param neighbours Neighbourhood around the Boid
 * @return Vector towards the center of the neighbourhood.
 */
QVector2D calculateCohesionVector(const Boid& boid, const std::vector<Boid>& neighbours);

/**
 * @brief Calculate the vector that repels a given Boids from the other boids wihtin
 * the neighbourhood to maintain a minimum distance between them.
 *
 * The separation vector is calculated by calculating the difference between the boid and each
 * neighbour, and weighing the difference based on the distance itself. The result is that the
 * separation force is greater with boids that are closer to each other.
 *
 * @param boid Boid to check calculate the cohesion vector for.
 * @param neighbours Neighbourhood around the Boid
 * @param minDist Minimum distance to retain between Boids.
 * @return Repelling vector.
 */
QVector2D calculateSeparationVector(const Boid& boid, const std::vector<Boid>& neighbours,
                                    const float minDist);

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

template <typename T> T generateRandomValue(const T minValue, const T maxValue) {
    std::random_device               rd;
    std::mt19937                     gen(rd());
    std::uniform_real_distribution<> distr(minValue, maxValue);
    return distr(gen);
}

/**
 * @brief Generate a random 2D vector with a maximum allowed madnitude.
 * @param maxMagnitude Maximum allowed length/madnitude.
 * @return Generated vector.
 */
QVector2D generateRandomVelocityVector(const float maxMagnitude);

std::vector<Boid> getBoidNeighbourhood(const Boid& boid, const std::vector<Boid>& flock,
                                       const float& dist);

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
 * @param vec Vector to sclae.
 * @param scalar Scalar value.
 * @return QVector2D scaled.
 */
QVector2D scaleVector(const QVector2D& vec, const float& scalar);

/**
 * @brief Wrap the position of a boid within a defined simulation area.
 * @param boid Boid to process.
 * @param rect Rectangle representing the simulation area.
 */
void wrapBoidPosition(Boid& boid, const QRectF& rect);

float wrapValue(const float& value, const float& minValue, const float& maxValue);

/**
 * @brief Clip the length of a vector so that it falls within an allowed minimum/maximum range. This
 * operation is performed in place on the input vector.
 * @param vec Vector to clip.
 * @param minMagnitude Minimum magnitude/length value of the vector.
 * @param maxMagnitude Maximum magnitude/length value of the vector.
 * @throws std::invalid_argument If the minimum value is greater than the maximum.
 */
void clipVectorMangitude(QVector2D& vec, const float& minMagnitude, const float& maxMagnitude);

} // namespace utils
}; // namespace boids
