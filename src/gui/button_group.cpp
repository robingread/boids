#include "button_group.h"

namespace ui {

ButtonGroup::ButtonGroup(QWidget* parent) : QWidget(parent) {

    qRegisterMetaType<boids::BoidType>("boids::BoidType");
    qRegisterMetaType<std::vector<boids::BoidType>>("std::vector<boids::BoidType>");

    m_clearAll   = new QPushButton("Clear All", this);
    m_clearBoids = new QPushButton("Clear Boids", this);
    m_clearObs   = new QPushButton("Clear Obstacles", this);
    m_clearPred  = new QPushButton("Clear Predators", this);

    m_layout = new QVBoxLayout(this);
    m_layout->addWidget(m_clearAll);
    m_layout->addWidget(m_clearBoids);
    m_layout->addWidget(m_clearObs);
    m_layout->addWidget(m_clearPred);

    QObject::connect(m_clearAll, &QPushButton::clicked, this, &ButtonGroup::onClearAllPressed);
    QObject::connect(m_clearBoids, &QPushButton::clicked, this, &ButtonGroup::onClearBoidsPressed);
    QObject::connect(m_clearObs, &QPushButton::clicked, this, &ButtonGroup::onClearObsPressed);
    QObject::connect(m_clearPred, &QPushButton::clicked, this, &ButtonGroup::onClearPredPressed);
}

void ButtonGroup::onClearAllPressed() {
    const std::vector<boids::BoidType> types = {boids::BOID, boids::OBSTACLE, boids::PREDATOR};
    emit                               clearBoids(types);
}

void ButtonGroup::onClearBoidsPressed() { emit clearBoids({boids::BOID}); }

void ButtonGroup::onClearObsPressed() { emit clearBoids({boids::OBSTACLE}); }

void ButtonGroup::onClearPredPressed() { emit clearBoids({boids::PREDATOR}); }

} // namespace ui