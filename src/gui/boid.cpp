#include "boid.h"
#include <QPainterPath>

Boid::Boid(const QPointF& pos, const float rot, const QColor& colour) {
    m_width  = 30;
    m_height = 20;
    m_colour = colour;
    this->setPos(pos);
    this->setRotation(rot);
}

QRectF Boid::boundingRect() const {
    qreal penWidth = 1;
    return QRectF(-(m_width * 0.5) - (penWidth / 2), -(m_height * 0.5) - (penWidth / 2),
                  m_width + penWidth, m_height + penWidth);
}

void Boid::paint(QPainter* painter, const QStyleOptionGraphicsItem* /*option*/,
                 QWidget* /*widget*/) {
    const int     w = m_width / 2;
    const int     h = m_height / 2;
    const QPointF p1(w, 0);
    const QPointF p2(-w, h);
    const QPointF p3(-w, -h);

    QPainterPath path;
    path.moveTo(p1);
    path.lineTo(p2);
    path.lineTo(p3);
    path.lineTo(p1);

    painter->setPen(Qt ::NoPen);
    painter->fillPath(path, QBrush(m_colour));
}
