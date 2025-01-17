#include "boids.h"
#include "utils.h"
#include <QtMath>
#include <math.h>

namespace boids {

Boid::Boid(const uint16_t& id, const BoidType type) : id_(id), type_(type) {
    position_.setX(0.0f);
    position_.setY(0.0f);
    velocity_.setX(0.0f);
    velocity_.setY(0.0f);
}

Boid::Boid(const uint16_t& id, const float x, const float y, const BoidType type)
    : id_(id), type_(type) {
    position_.setX(x);
    position_.setY(y);

    const float maxVelocity = 5.0;
    velocity_               = utils::generateRandomVelocityVector(maxVelocity);

    const int r = utils::generateRandomValue<int>(50, 200);
    const int g = utils::generateRandomValue<int>(100, 255);
    const int b = utils::generateRandomValue<int>(100, 255);
    color_      = QColor(r, g, b, 255);
}

Boid::Boid(const uint16_t& id, const float x, const float y, const float dx, const float dy,
           const BoidType type)
    : id_(id), type_(type) {
    position_.setX(x);
    position_.setY(y);
    velocity_.setX(dx);
    velocity_.setY(dy);
}

float Boid::getAngle() const { return std::atan2(velocity_.y(), velocity_.x()); }

QColor Boid::getColor() const { return color_; }

uint16_t Boid::getId() const { return id_; }

QPointF Boid::getPosition() const { return position_; }

BoidType Boid::getType() const { return type_; }

QVector2D Boid::getVelocity() const { return velocity_; }

void Boid::setColor(const QColor& colour) { color_ = colour; }

void Boid::setPosition(const QPointF& pos) { position_ = pos; }

void Boid::setVelocity(const QVector2D& vel) { velocity_ = vel; }

}; // namespace boids
