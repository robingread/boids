#pragma once

#include <QColor>
#include <QPoint>
#include <QVector2D>

namespace boids {

/**
 * @brief The type of Boid:
 * - BOID is a standard boids that follows normal rules.
 * - PREDATOR is a boids that hunts the BOIDs, but avoids other PREDATORs.
 * - OBSTACLE is a static obstacle that both BOIDs and PREDATORs avoid.
 */
enum BoidType { BOID, PREDATOR, OBSTACLE };

/**
 * @brief The Boid class contains the data pertaining to a single Boid.
 * That is, the ID, the colour, the current position and heading and the current velocity.
 */
class Boid {
  public:
    /**
     * @brief Construct a new Boid object at location 0.0, 0.0, with not velocity.
     * @param id ID to assign to the Boid.
     */
    Boid(const uint16_t& id);

    /**
     * @brief Construct a new Boid object at a given location with a given ID.
     * @param id ID to assign to the Boid.
     * @param x X screen coordinate.
     * @param y Y screen coordinate.
     */
    Boid(const uint16_t& id, const float x, const float y);

    /**
     * @brief Construct a new Boid object at a given location, with a given velocity and ID.
     *
     * @param id ID to assign to the Boid.
     * @param x X screen coordinate.
     * @param y Y screen coordinate.
     * @param dx X velocity.
     * @param dy Y velocity.
     */
    Boid(const uint16_t& id, const float x, const float y, const float dx, const float dy);

    /**
     * @brief Get the heading angle of the boid. This will be in the range (-PI, PI) and
     * is calculated from the Boid's velocity vector..
     * @return Heading angle in radians.
     */
    float getAngle() const;

    /**
     * @brief Get the RGB colour of the Boid
     * @return Colour
     */
    QColor getColor() const;

    /**
     * @brief Get the Boid ID.
     * @return Boid ID.
     */
    uint16_t getId() const;

    /**
     * @brief Get the current position of the Boid.
     * @return Position in the scene.
     */
    QPointF getPosition() const;

    /**
     * @brief Get the current velocity of the Boid.
     * @return Velocity in the scene.
     */
    QVector2D getVelocity() const;

    /**
     * @brief Set the Color of the Boid.
     * @param color New colour to set.
     */
    void setColor(const QColor& color);

    /**
     * @brief Set the Position of the Boid.
     * @param pos Position in the format (x, y).
     */
    void setPosition(const QPointF& pos);

    /**
     * @brief Set the Velocity of the Boid.
     * @param pos Velocity in the format (vx, vy).
     */
    void setVelocity(const QVector2D& vel);

  private:
    uint16_t  m_id;
    QColor    m_color;
    QPointF   m_position;
    QVector2D m_velocity;
};

}; // namespace boids
