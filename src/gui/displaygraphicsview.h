#ifndef DISPLAYGRAPHICSVIEW_H
#define DISPLAYGRAPHICSVIEW_H

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

  signals:
    void createItem(const QPointF pos, const boids::BoidType& type = boids::BoidType::BOID);

  public slots:
    void mousePressEvent(QMouseEvent* event) override;
    void renderBoids(const QList<boids::Boid>& boids);
};

#endif // DISPLAYGRAPHICSVIEW_H
