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


template<typename T>
T generateRandomValue(const T minValue, const T maxValue)
{
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> distr(minValue, maxValue);
    return distr(gen);
}


QVector2D generateRandomVelocityVector(const float maxMagnitude);


std::vector<Boid> getBoidNeighbourhood(
        const Boid &boid,
        const std::vector<Boid> &flock,
        const float &dist);


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
