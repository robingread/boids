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
    void createBoid(const QPointF& pos, const boids::BoidType& type = boids::BoidType::BOID);
};
