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

DisplayGraphicsView::DisplayGraphicsView(QWidget* parent) : QGraphicsView(parent) {
    m_scene = new QGraphicsScene();
    this->setSceneRect(0, 0, this->width(), this->height());
    this->setScene(m_scene);

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

void DisplayGraphicsView::addDisplayItem(const QPointF& pos) {
    const float angle = generateRandomValue<float>(-180.0f, 180.0f);

    const int    r = generateRandomValue<int>(0, 150);
    const int    g = generateRandomValue<int>(0, 255);
    const int    b = generateRandomValue<int>(50, 255);
    const QColor color(r, g, b, 255);

    addDisplayItem(pos, angle, color);
}

void DisplayGraphicsView::addDisplayItem(const QPointF& pos, const float& angle,
                                         const QColor& color) {
    Boid* boid = new Boid(pos, angle, color);
    m_scene->addItem(boid);
}

void DisplayGraphicsView::renderObstacle(const QPointF& pos) {
    const float           diameter = 15;
    const float           x        = pos.x() - (diameter * 0.5f);
    const float           y        = pos.y() - (diameter * 0.5f);
    QGraphicsEllipseItem* circle   = new QGraphicsEllipseItem(x, y, diameter, diameter);
    circle->setBrush(QBrush(Qt::lightGray));
    circle->setPen(Qt::NoPen);
    m_scene->addItem(circle);
}

void DisplayGraphicsView::renderBoids(const QList<boids::Boid>& boids) {
    m_scene->clear();
    for (const boids::Boid& b : boids) {
        switch (b.getType()) {
            case boids::BoidType::BOID:
                addDisplayItem(b.getPosition(), qRadiansToDegrees(b.getAngle()), b.getColor());
                break;
            case boids::OBSTACLE:
                renderObstacle(b.getPosition());
                break;
            default:
                break;
        }
    }
}
