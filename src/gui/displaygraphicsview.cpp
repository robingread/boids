#include "displaygraphicsview.h"
#include "boid.h"
#include "boids.h"
#include <QMouseEvent>
#include <QtMath>
#include <iostream>
#include <random>

template <typename T> T generateRandomValue(const T minValue, const T maxValue) {
    std::random_device              rd;        // obtain a random number from hardware
    std::mt19937                    gen(rd()); // seed the generator
    std::uniform_int_distribution<> distr(minValue, maxValue);
    return distr(gen);
}

namespace ui {

DisplayGraphicsView::DisplayGraphicsView(QWidget* parent) : QGraphicsView(parent) {
    m_scene = new QGraphicsScene();
    this->setSceneRect(0, 0, this->width(), this->height());
    this->setScene(m_scene);

    m_displayItems.clear();

    const int c = 60;
    m_scene->setBackgroundBrush(QBrush(QColor(c, c, c, 255)));
}

void DisplayGraphicsView::mousePressEvent(QMouseEvent* event) {
    if (event->button() == Qt::LeftButton) {
        std::cout << "Left Mouse Button Clicked!" << std::endl;
        emit createItem(mapToScene(event->pos()), boids::BoidType::BOID);
    } else if (event->button() == Qt::RightButton) {
        std::cout << "Right Mouse Button Clicked!" << std::endl;
        emit createItem(mapToScene(event->pos()), boids::BoidType::PREDATOR);
    } else if (event->button() == Qt::MiddleButton) {
        std::cout << "Middle Mouse Button Clicked!" << std::endl;
        emit createItem(mapToScene(event->pos()), boids::BoidType::OBSTACLE);
    }
}

void DisplayGraphicsView::renderBoid(const uint16_t& id, const QPointF& pos, const float& angle,
                                     const QColor& color) {

    if (!m_displayItems.contains(id)) {
        m_displayItems[id] = std::make_unique<Boid>(pos, angle, color);
        m_scene->addItem(m_displayItems[id].get());
    }
    m_displayItems[id]->setPos(pos);
    m_displayItems[id]->setRotation(angle);
}

void DisplayGraphicsView::renderCircle(const QPointF& pos, const float& radius) {
    const float           x      = pos.x() - (radius * 0.5f);
    const float           y      = pos.y() - (radius * 0.5f);
    QGraphicsEllipseItem* circle = new QGraphicsEllipseItem(x, y, 100.0, 100.0);
    circle->setPen(QPen(Qt::red));
    m_scene->addItem(circle);
}

void DisplayGraphicsView::renderObstacle(const uint16_t& id, const QPointF& pos) {
    if (!m_displayItems.contains(id)) {
        m_displayItems[id] = std::make_unique<Obstacle>(pos, QColor(Qt::lightGray), 8.0f);
        m_scene->addItem(m_displayItems[id].get());
    }
}

void DisplayGraphicsView::renderPredator(const uint16_t& id, const QPointF& pos,
                                         const float& angle) {
    if (!m_displayItems.contains(id)) {
        m_displayItems[id] = std::make_unique<Predator>(pos, angle, QColor(Qt::red));
        m_scene->addItem(m_displayItems[id].get());
    }
    m_displayItems[id]->setPos(pos);
    m_displayItems[id]->setRotation(angle);
}

void DisplayGraphicsView::renderBoids(const QList<boids::Boid>& boids) {
    for (const boids::Boid& b : boids) {
        switch (b.getType()) {
            case boids::BoidType::BOID:
                renderBoid(b.getId(), b.getPosition(), qRadiansToDegrees(b.getAngle()),
                           b.getColor());
                break;
            case boids::OBSTACLE:
                renderObstacle(b.getId(), b.getPosition());
                break;
            case boids::PREDATOR:
                renderPredator(b.getId(), b.getPosition(), qRadiansToDegrees(b.getAngle()));
                break;
            default:
                break;
        }
    }
}

} // namespace ui
