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

QColor calculateBoidColor(const Boid& boid, const std::vector<Boid>& neighbours) {

    const QColor& boidColor = boid.getColor();

    // If there are no neighbours, return the current color.
    if (neighbours.size() == 0) {
        return boidColor;
    }

    // Calculate the average hue difference to all the neighbours.
    float h = 0.0f;
    for (const Boid& n : neighbours) {
        const float dist = distanceBetweenBoids(boid, n);

        const float h1      = boidColor.hsvHue();
        const float h2      = n.getColor().hue();
        const float hueDiff = shortestDistanceInWrapedSpace(h2, h1, 0.0f, 359.0f);
        h += (hueDiff / dist);
    }
    h /= neighbours.size();

    // Calculate a small correction to the boid hue to move it closer to the group average, with
    // some noise.
    const float noise  = generateRandomValue<float>(-3.0f, 3.0f) * 0.0001f;
    float       newHue = float(boidColor.hue()) - (h * 0.005f) + noise;

    // Make sure that the new hue wraps into the range [0, 359].
    newHue = wrapValue(newHue, 0.0f, 359.0f);

    // Create and return hue in new QColor object.
    QColor ret;
    ret.setHsv(int(newHue), boidColor.saturation(), boidColor.value());
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
    const auto p1 = b1.getPosition();
    const auto p2 = b2.getPosition();

    const float dx =
        std::abs(shortestDistanceInWrapedSpace(p1.x(), p2.x(), bounds.left(), bounds.right()));
    const float dy =
        std::abs(shortestDistanceInWrapedSpace(p1.y(), p2.y(), bounds.top(), bounds.bottom()));

    return std::sqrt(std::pow(dx, 2) + std::pow(dy, 2));
}

QVector2D generateRandomVelocityVector(const float maxMagnitude) {
    const float dx = generateRandomValue<float>(-1.0f, 1.0f);
    const float dy = generateRandomValue<float>(-1.0f, 1.0f);
    const float w  = generateRandomValue<float>(0.0f, maxMagnitude);
    return scaleVector(QVector2D(dx, dy), w);
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

float shortestDistanceInWrapedSpace(const float& v1, const float& v2, const float& min,
                                    const float& max) {
    // Standard difference
    const float a = v2 - v1;

    // Difference that is wrapped around
    const float b = (std::min(v1, v2) - min) + (max - std::max(v1, v2));

    if (std::abs(a) <= std::abs(b))
        return a;

    return b * (-a / std::abs(a));
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
