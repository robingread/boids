#pragma once

#include "boid.h"
#include <boids.h>

#include <QGraphicsScene>
#include <QGraphicsView>
#include <QList>
#include <QObject>

class DisplayGraphicsView : public QGraphicsView {
    Q_OBJECT

  public:
    explicit DisplayGraphicsView(QWidget* parent = 0);

  private:
    QGraphicsScene* m_scene;

    /**
     * @brief Render a standard Boid.
     * @param pos Postion of the Boid.
     * @param angle Heading of the Boid.
     */
    void renderBoid(const uint16_t& id, const QPointF& pos, const float& angle,
                    const QColor& color);

    /**
     * @brief Render a circle with a given radius around a point.
     * @param pos Center of the circle (in pixels).
     * @param radius Radius of the cirlce (in pixels).
     */
    void renderCircle(const QPointF& pos, const float& radius);

    /**
     * @brief Render an Obstacle Boid at a given position.
     * @param pos Position of the obstacle.
     */
    void renderObstacle(const uint16_t& id, const QPointF& pos);

    /**
     * @brief Render a Predator Boid.
     * @param pos Postion of the Boid.
     * @param angle Heading of the Boid.
     */
    void renderPredator(const uint16_t& id, const QPointF& pos, const float& angle);

  private:
    std::map<const uint16_t, std::unique_ptr<QGraphicsItem>> m_displayItems;

  signals:
    void createItem(const QPointF pos, const boids::BoidType& type = boids::BoidType::BOID);

  public slots:
    void mousePressEvent(QMouseEvent* event) override;
    void renderBoids(const QList<boids::Boid>& boids);
};
