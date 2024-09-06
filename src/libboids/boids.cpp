#include "boids.h"
#include "utils.h"
#include <math.h>
#include <QtMath>


namespace boids {

Boid::Boid(const uint16_t &id) : m_id(id)
{
    m_position.setX(0.0f);
    m_position.setY(0.0f);
    m_velocity.setX(0.0f);
    m_velocity.setY(0.0f);
}


Boid::Boid(const uint16_t &id, const float x, const float y)  : m_id(id)
{
    m_position.setX(x);
    m_position.setY(y);

    const float maxVelocity = 5.0;
    m_velocity = utils::generateRandomVelocityVector(maxVelocity);

    const int r = utils::generateRandomValue<int>(50, 200);
    const int g = utils::generateRandomValue<int>(100, 255);
    const int b = utils::generateRandomValue<int>(100, 255);
    m_color = QColor(r, g, b, 255);
}


Boid::Boid(const uint16_t &id, const float x, const float y, const float dx, const float dy)  : m_id(id)
{
    m_position.setX(x);
    m_position.setY(y);
    m_velocity.setX(dx);
    m_velocity.setY(dy);
}


float Boid::getAngle() const
{
    return std::atan2(m_velocity.y(), m_velocity.x());
}


QColor Boid::getColor() const
{
    return m_color;
}


uint16_t Boid::getId() const
{
    return m_id;
}


QPointF Boid::getPosition() const
{
    return m_position;
}


QVector2D Boid::getVelocity() const
{
    return m_velocity;
}


void Boid::setPosition(const QPointF &pos)
{
    m_position = pos;
}

void Boid::setVelocity(const QVector2D &vel)
{
    m_velocity = vel;
}

};
