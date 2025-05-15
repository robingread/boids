#pragma once

#include "boids.h"
#include "control_panel.h"
#include "displaygraphicsview.h"
#include "simthread.h"
#include <QHBoxLayout>
#include <QMainWindow>

class Dialog : public QMainWindow {
    Q_OBJECT

  public:
    Dialog(QWidget* parent = nullptr);
    ~Dialog();

    void run();

  protected:
    void resizeEvent(QResizeEvent* event) override;

  private:
    SimThread*               m_sim;
    ui::ControlPanelWidget*  m_control;
    ui::DisplayGraphicsView* m_graphicsView;
    QHBoxLayout*             m_layout;

    std::shared_ptr<boids::Flock> m_flock;

  private slots:
    void onConfigChanged();

    /**
     * @brief Add multiple boids to the simulation.
     *
     * These boids will be created at random positions within the current display scene.
     *
     * @param count The number of boids to create.
     */
    void addBoids(const std::size_t count);

    /**
     * @brief Create/add a of a specific type at a specific location.
     *
     * This handles creating boids based on mouse clicks on screen.
     *
     * @param pos Position to create the boid at in the QGraphicsView scene rect.
     * @param type The type of Boid to create.
     */
    void createBoid(const QPointF& pos, const boids::BoidType& type = boids::BoidType::BOID);

    /**
     * @brief Clear all the boids of a given type.
     * @param types The types of boids to clear.
     */
    void clearBoids(const std::vector<boids::BoidType>& types);
};
