#include "utils.h"
#include "boids.h"
#include <math.h>

namespace boids {
namespace utils {

QVector2D calculateAlignmentVector(const Boid& boid, const std::vector<Boid>& neighbours) {
    if (neighbours.size() == 0) {
        return QVector2D(0.0f, 0.0f);
    }

    QVector2D vec(0.0, 0.0);
    for (const Boid& n : neighbours) {
        const float dist = distanceBetweenBoids(boid, n);
        QVector2D   vel  = n.getVelocity().normalized() / dist;
        vec += vel;
    }

    return vec;
}

QVector2D calculateCohesionVector(const Boid& boid, const std::vector<Boid>& neighbours) {
    if (neighbours.size() == 0) {
        return QVector2D(0.0f, 0.0f);
    }

    QVector2D vec(0.0, 0.0);
    for (const Boid& n : neighbours) {
        vec += QVector2D(n.getPosition());
    }

    vec /= float(neighbours.size());
    QVector2D ret = vec - QVector2D(boid.getPosition());
    ret.normalize();
    ret *= 0.25f;
    return ret;
}

QVector2D calculateSeparationVector(const Boid& boid, const std::vector<Boid>& neighbours,
                                    const float minDist) {
    if (neighbours.size() == 0) {
        return QVector2D(0.0f, 0.0f);
    }

    QVector2D vec(0.0, 0.0);
    for (const Boid& n : neighbours) {
        const float dist = distanceBetweenBoids(boid, n);

        if ((dist > minDist))
            continue;

        else if (dist == 0.0f) {
            vec += QVector2D(1.0f, 0.0f);
            continue;
        }

        const float     w    = std::max(1.0f, dist - minDist);
        const QVector2D diff = QVector2D(boid.getPosition() - n.getPosition()).normalized() / w;
        vec += diff;
    }

    return vec;
}

float distanceBetweenBoids(const Boid& b1, const Boid& b2) {
    const auto  p1   = b1.getPosition();
    const auto  p2   = b2.getPosition();
    const float dx   = std::pow(p1.x() - p2.x(), 2);
    const float dy   = std::pow(p1.y() - p2.y(), 2);
    const float dist = std::sqrt(dx + dy);
    return dist;
}

float distanceBetweenBoids(const Boid& b1, const Boid& b2, const QRectF& bounds) {
    const float dx1 = std::abs(b1.getPosition().x() - b2.getPosition().x());
    const float dx2 = std::abs((bounds.right() - b1.getPosition().x()) + b2.getPosition().x());
    const float dx3 = std::abs(b1.getPosition().x() + (bounds.right() - b2.getPosition().x()));

    const float dy1 = std::abs(b1.getPosition().y() - b2.getPosition().y());
    const float dy2 = std::abs((bounds.bottom() - b1.getPosition().y()) + b2.getPosition().y());
    const float dy3 = std::abs(b1.getPosition().y() + (bounds.bottom() - b2.getPosition().y()));

    const float dx = std::min(std::min(dx1, dx2), dx3);
    const float dy = std::min(std::min(dy1, dy2), dy3);

    return std::sqrt(std::pow(dx, 2) + std::pow(dy, 2));
}

QVector2D generateRandomVelocityVector(const float maxMagnitude) {
    const float dx = generateRandomValue<float>(-1.0f, 1.0f);
    const float dy = generateRandomValue<float>(-1.0f, 1.0f);
    const float w  = generateRandomValue<float>(0.1f, maxMagnitude);
    return scaleVector(QVector2D(dx, dy), w);
}

std::vector<Boid> getBoidNeighbourhood(const Boid& boid, const std::vector<boids::Boid>& flock,
                                       const float& dist) {
    // TODO: This does not take into account the issue of wrap around with the sim space.
    std::vector<Boid> ret;
    for (const Boid& b : flock) {
        if (boid.getId() == b.getId())
            continue;
        const float d = distanceBetweenBoids(boid, b);
        if (d > dist)
            continue;
        ret.push_back(b);
    }
    return ret;
}

std::vector<Boid> getBoidNeighbourhood(const Boid& boid, const std::vector<boids::Boid>& flock,
                                       const float& dist, const QRectF& bounds) {
    std::vector<Boid> ret;
    for (const Boid& b : flock) {
        if (boid.getId() == b.getId())
            continue;
        const float d = distanceBetweenBoids(boid, b, bounds);
        if (d > dist)
            continue;
        ret.push_back(b);
    }
    return ret;
}

std::size_t getTotalNumBoids(const std::map<BoidType, std::vector<Boid>>& boids) {
    std::size_t n = 0;
    for (const auto& [key, value] : boids) {
        n += value.size();
    }
    return n;
}

QVector2D scaleVector(const QVector2D& vec, const float& scalar) {
    return vec.normalized() * scalar;
}

void wrapBoidPosition(Boid& boid, const QRectF& rect) {
    const float x = wrapValue(boid.getPosition().x(), rect.left(), rect.right());
    const float y = wrapValue(boid.getPosition().y(), rect.top(), rect.bottom());
    boid.setPosition(QPointF(x, y));
}

float wrapValue(const float& value, const float& minValue, const float& maxValue) {
    const float r     = value - minValue;
    const float range = maxValue - minValue;
    if (r >= 0.0) {
        return minValue + std::fmod(r, range);
    } else {
        return maxValue + std::fmod(r, range);
    }
}

void clipVectorMangitude(QVector2D& vec, const float& maxMagnitude) {
    if (vec.length() < maxMagnitude)
        return;
    vec *= maxMagnitude / vec.length();
}

void clipVectorMangitude(QVector2D& vec, const float& minMagnitude, const float& maxMagnitude) {

    if (minMagnitude > maxMagnitude)
        throw std::invalid_argument("Minimum value is greater than the maximum.");

    if (vec.length() < minMagnitude) {
        vec *= minMagnitude / vec.length();
        return;
    }

    if (vec.length() < maxMagnitude)
        return;
    vec *= maxMagnitude / vec.length();
}

} // namespace utils
}; // namespace boids
