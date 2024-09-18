#pragma once

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

    void addDisplayItem(const QPointF& pos);
    void addDisplayItem(const QPointF& pos, const float& angle, const QColor& color);

    /**
     * @brief Render an Obstacle Boid at a given position.
     * @param pos Position of the obstacle.
     */
    void renderObstacle(const QPointF& pos);

    /**
     * @brief Render a Predator Boid.
     * @param pos Postion of the Boid.
     * @param angle Heading of the Boid.
     */
    void renderPredator(const QPointF& pos, const float& angle);

  signals:
    void createItem(const QPointF pos, const boids::BoidType& type = boids::BoidType::BOID);

  public slots:
    void mousePressEvent(QMouseEvent* event) override;
    void renderBoids(const QList<boids::Boid>& boids);
};
