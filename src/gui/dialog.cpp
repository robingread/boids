#include "dialog.h"
#include "displaygraphicsview.h"
#include "ui_dialog.h"
#include <boids.h>
#include <iostream>

Q_DECLARE_METATYPE(QList<boids::Boid>);

Dialog::Dialog(QWidget* parent) : QDialog(parent), ui(new Ui::Dialog) {
    ui->setupUi(this);

    qRegisterMetaType<QList<boids::Boid>>("QList<boids::Boid>");
}

Dialog::~Dialog() {
    m_sim.stopSim();
    m_sim.quit();
    m_sim.wait();
    delete ui;
}

void Dialog::run() {
    QObject::connect(ui->graphicsView, &DisplayGraphicsView::createItem, &m_sim,
                     &SimThread::addNewItem);

    QObject::connect(&m_sim, &SimThread::update, ui->graphicsView,
                     &DisplayGraphicsView::renderBoids);

    const QRectF rect = ui->graphicsView->mapToScene(ui->graphicsView->rect()).boundingRect();
    m_sim.updateSceneBounds(rect);

    m_sim.start();
}
