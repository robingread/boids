#include "boid.h"
#include <QPainterPath>

Boid::Boid(const QPointF& pos, const float rot, const QColor& colour) {
    m_width  = 15;
    m_height = 10;
    m_colour = colour;
    this->setPos(pos);
    this->setRotation(rot);

    const float   w = m_width / 2.0f;
    const float   h = m_height / 2.0f;
    const QPointF p1(w, 0);
    const QPointF p2(-w, h);
    const QPointF p3(-w, -h);
    m_path.moveTo(p1);
    m_path.lineTo(p2);
    m_path.lineTo(p3);
    m_path.lineTo(p1);

    const qreal penWidth = 1;
    m_boundingRect = QRectF(-(m_width * 0.5) - (penWidth / 2), -(m_height * 0.5) - (penWidth / 2),
                            m_width + penWidth, m_height + penWidth);
}

QRectF Boid::boundingRect() const { return m_boundingRect; }

void Boid::paint(QPainter* painter, const QStyleOptionGraphicsItem* /*option*/,
                 QWidget* /*widget*/) {
    painter->setRenderHint(QPainter::Antialiasing, false);
    painter->setPen(Qt::NoPen);
    painter->fillPath(m_path, QBrush(m_colour));
}

void Boid::setColor(const QColor& color) { m_colour = color; }

Obstacle::Obstacle(const QPointF& pos, const QColor& color, const float& radius)
    : Boid(pos, 0.0f, color) {
    const float xy = -radius;
    const float wh = radius * 2.0;
    m_boundingRect = QRectF(xy, xy, wh, wh);
    this->setPos(pos);
};

// QRectF Obstacle::boundingRect() const { return m_boundingRect; }

void Obstacle::paint(QPainter* painter, const QStyleOptionGraphicsItem* /*option*/,
                     QWidget* /*widget*/) {
    painter->setRenderHint(QPainter::Antialiasing, false);
    painter->setBrush(QBrush(m_colour));
    painter->setPen(Qt::NoPen);
    painter->drawEllipse(m_boundingRect);
}

Predator::Predator(const QPointF& pos, const float rot, const QColor& colour)
    : Boid(pos, rot, colour) {
    m_width *= 1.5;
    m_height *= 1.5;

    const float   w = m_width / 2.0f;
    const float   h = m_height / 2.0f;
    const QPointF p1(w, 0);
    const QPointF p2(-w, h);
    const QPointF p3(-w * 0.6, 0);
    const QPointF p4(-w, -h);

    m_path.clear();
    m_path.moveTo(p1);
    m_path.lineTo(p2);
    m_path.lineTo(p3);
    m_path.lineTo(p4);
    m_path.lineTo(p1);

    const qreal penWidth = 1;
    m_boundingRect = QRectF(-(m_width * 0.5) - (penWidth / 2), -(m_height * 0.5) - (penWidth / 2),
                            m_width + penWidth, m_height + penWidth);
};

void Predator::paint(QPainter* painter, const QStyleOptionGraphicsItem* /*option*/,
                     QWidget* /*widget*/) {
    // painter->setRenderHint(QPainter::Antialiasing, false);
    painter->setPen(Qt::NoPen);
    painter->fillPath(m_path, QBrush(m_colour));
}
