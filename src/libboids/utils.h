#pragma once

#include "boids.h"
#include <random>
#include <vector>
#include <QRectF>


namespace boids {
namespace utils {

/**
 * @brief Calculate the average velocity vector for a neighbourhood of Boids.
 * @param boid Not used.
 * @param neighbours Neighbour
 * @return QVector2D 
 */
QVector2D calculateAlignmentVector(const Boid &boid, const std::vector<Boid> &neighbours);


/**
 * @brief Calculate the vector that pulls a Boid towards the center of the neighbourood.
 * @param boid Boid to check calculate the cohesion vector for.
 * @param neighbours Neighbourhood around the Boid
 * @return Vector towards the center of the neighbourhood.
 */
QVector2D calculateCohesionVector(const Boid &boid, const std::vector<Boid> &neighbours);


/**
 * @brief Calculate the vector that repels a given Boids from the other boids wihtin 
 * the neighbourhood to maintain a minimum distance between them.
 * @param boid Boid to check calculate the cohesion vector for.
 * @param neighbours Neighbourhood around the Boid
 * @param minDist Minimum distance to retian between Boids.
 * @return Repelling vector.
 */
QVector2D calculateSeparationVector(
    const Boid &boid,
    const std::vector<Boid> &neighbours,
    const float minDist);


float calculateRepulsionWeight(const float dist, const float minDist);

/**
 * @brief Calculate the euclidean distance between two Boids.
 * @param b1 First boid.
 * @param b2 Second boid.
 * @return Euclidean distance.
 */
float distanceBetweenBoids(const Boid &b1, const Boid &b2);


/**
 * @brief Calculate the alternative Euclidean distance between two boids, which looks at the 
 * distance across the scene bounds (as the scene is wrapped).
 * @param b1 First boid.
 * @param b2 Second boid.
 * @param bounds Bounds of the scene.
 * @return Euclidean distance.
 */
float distanceBetweenBoids(const Boid &b1, const Boid &b2, const QRectF &bounds);

template<typename T>
T generateRandomValue(const T minValue, const T maxValue)
{
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> distr(minValue, maxValue);
    return distr(gen);
}

/**
 * @brief Generate a random 2D vector with a maximum allowed madnitude.
 * @param maxMagnitude Maximum allowed length/madnitude.
 * @return Generated vector.
 */
QVector2D generateRandomVelocityVector(const float maxMagnitude);


std::vector<Boid> getBoidNeighbourhood(
        const Boid &boid,
        const std::vector<Boid> &flock,
        const float &dist);

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
std::vector<Boid> getBoidNeighbourhood(
        const Boid &boid,
        const std::vector<Boid> &flock,
        const float &dist,
        const QRectF &bounds);

/**
 * @brief Get a QVector scaled by a given value.
 * @param vec Vector to sclae.
 * @param scalar Scalar value.
 * @return QVector2D scaled.
 */
QVector2D scaleVector(const QVector2D &vec, const float &scalar);


/**
 * @brief Wrap the position of a boid within a defined simulation area.
 * @param boid Boid to process.
 * @param rect Rectangle representing the simulation area.
 */
void wrapBoidPosition(Boid &boid, const QRectF &rect);


float wrapValue(const float &value, const float &minValue, const float &maxValue);

/**
 * @brief Clip the length of a vector to a given value. This operation is performed 
 * in place on the input vector.
 * @param vec Vector to clip.
 * @param maxMagnitude Maximum magdinute/length value of the vector.
 */
void clipVectorMangitude(QVector2D &vec, const float &maxMagnitude);

}};
