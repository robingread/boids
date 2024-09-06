#ifndef BOID_H
#define BOID_H

#include <QColor>
#include <QGraphicsItem>
#include <QPainter>
#include <QPointF>

class Boid : public QGraphicsItem {
  public:
    Boid(const QPointF& pos, const float rot, const QColor& colour);
    QRectF boundingRect() const override;
    void paint(QPainter* painter, const QStyleOptionGraphicsItem* option, QWidget* widget) override;

  private:
    QColor      m_colour;
    std::size_t m_width;
    std::size_t m_height;
};

#endif // BOID_H
