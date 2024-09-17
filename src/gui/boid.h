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

  protected:
    QColor      m_colour;
    std::size_t m_width;
    std::size_t m_height;
};

