#pragma once

#include <QColor>
#include <QGraphicsItem>
#include <QPainter>
#include <QPointF>

/**
 * @brief The Boid class is used to render Boid in the GUI. It inherits from the
 * QGraphicsItem class and thus can be added to a QGraphicsScene but provides a custom
 * implementation for the boundingRect() and paint() methods.
 */
class Boid : public QGraphicsItem {
  public:
    Boid(const QPointF& pos, const float rot, const QColor& colour);
    QRectF boundingRect() const override;
    void paint(QPainter* painter, const QStyleOptionGraphicsItem* option, QWidget* widget) override;

    /**
     * @brief Set the colour fo the Boid object.
     * @param color The QColor to set.
     */
    void setColor(const QColor& color);

  protected:
    QColor       m_colour;       ///< Colour og the boid.
    std::size_t  m_width;        ///< Width of the boid in pixels.
    std::size_t  m_height;       ///< Height of the boid in pixels.
    QPainterPath m_path;         ///< Path that draws the boid outline.
    QRectF       m_boundingRect; ///< Bounding rectangle for rendering
};

/**
 * @brief The Obstacle class is used to render Obstacles in the GUI and inherits from the
 * QGraphicsItem class.
 */
class Obstacle : public Boid {
  public:
    /**
     * @brief Construct a new Obstacle object.
     * @param pos Position of the obstacle.
     * @param color Colour of the obstacle.
     * @param radius Radius of the obstacle.
     */
    Obstacle(const QPointF& pos, const QColor& color, const float& radius);
    void paint(QPainter* painter, const QStyleOptionGraphicsItem* option, QWidget* widget) override;
};

/**
 * @brief The Predator class is used to render Predators in the GUI. It inherits from the
 * Boid class, which itself is a QGraphicsItem. The main differece from the Boid class is
 * that it has an alternative implementation for the paint() method.
 */
class Predator : public Boid {
  public:
    /**
     * @brief Construct a new Predator object.
     *
     * @param pos Position of the boid.
     * @param rot Rotation of the boid (in degrees).
     * @param color Colour of the boid.
     */
    Predator(const QPointF& pos, const float rot, const QColor& color);
    void paint(QPainter* painter, const QStyleOptionGraphicsItem* option, QWidget* widget) override;
};
