#include "dialog.h"
#include "displaygraphicsview.h"
#include <boids.h>
#include <iostream>

Q_DECLARE_METATYPE(QList<boids::Boid>);

Dialog::Dialog(QWidget* parent) : QMainWindow(parent) {

    m_sim = new SimThread(this);

    qRegisterMetaType<QList<boids::Boid>>("QList<boids::Boid>");

    m_control = new ui::ControlPanelWidget(this);
    m_control->setFixedWidth(300);

    m_graphicsView = new ui::DisplayGraphicsView(this);
    m_graphicsView->setMinimumSize(781, 581);

    m_layout = new QHBoxLayout(this);
    m_layout->addWidget(m_control);
    m_layout->addWidget(m_graphicsView);

    QWidget* window = new QWidget(this);
    window->setLayout(m_layout);

    this->setCentralWidget(window);
}

Dialog::~Dialog() {
    m_sim->stopSim();
    m_sim->quit();
    m_sim->wait();
}

void Dialog::resizeEvent(QResizeEvent* event) {
    QMainWindow::resizeEvent(event);
    const QRectF rect = m_graphicsView->mapToScene(m_graphicsView->rect()).boundingRect();
    m_sim->updateSceneBounds(rect);
}

void Dialog::onConfigChanged() {
    const auto boidCfg = m_control->m_boidCfgGroup->getConfig();
    const auto predCfg = m_control->m_predatorCfgGroup->getConfig();
    m_sim->setConfig(boidCfg, predCfg);
}

void Dialog::run() {
    QObject::connect(m_graphicsView, &ui::DisplayGraphicsView::createItem, m_sim,
                     &SimThread::addNewItem);

    QObject::connect(m_sim, &SimThread::update, m_graphicsView,
                     &ui::DisplayGraphicsView::renderBoids);

    QObject::connect(m_control->m_boidCfgGroup.get(), &ui::ConfigGroup::configChanged, this,
                     &Dialog::onConfigChanged);

    QObject::connect(m_control->m_predatorCfgGroup.get(), &ui::ConfigGroup::configChanged, this,
                     &Dialog::onConfigChanged);

    const QRectF rect = m_graphicsView->mapToScene(m_graphicsView->rect()).boundingRect();
    m_sim->updateSceneBounds(rect);

    m_sim->start();
}
