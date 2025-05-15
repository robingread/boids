#include "button_group.h"

namespace ui {

ButtonGroup::ButtonGroup(QWidget* parent) : QWidget(parent) {

    qRegisterMetaType<boids::BoidType>("boids::BoidType");
    qRegisterMetaType<std::vector<boids::BoidType>>("std::vector<boids::BoidType>");

    // Create the buttons for adding more boids
    add_boids_1_ = new QPushButton("+10 Boids", this);
    add_boids_2_ = new QPushButton("+50 Boids", this);
    add_boids_3_ = new QPushButton("+100 Boids", this);

    add_boids_button_layout_ = new QHBoxLayout();
    add_boids_button_layout_->addWidget(add_boids_1_);
    add_boids_button_layout_->addWidget(add_boids_2_);
    add_boids_button_layout_->addWidget(add_boids_3_);

    QObject::connect(add_boids_1_, &QPushButton::clicked, this, &ButtonGroup::onAddBoids1);
    QObject::connect(add_boids_2_, &QPushButton::clicked, this, &ButtonGroup::onAddBoids2);
    QObject::connect(add_boids_3_, &QPushButton::clicked, this, &ButtonGroup::onAddBoids3);

    // Create the buttons for clearing the different types of boids
    m_layout     = new QVBoxLayout(this);
    m_clearAll   = new QPushButton("Clear All", this);
    m_clearBoids = new QPushButton("Clear Boids", this);
    m_clearObs   = new QPushButton("Clear Obstacles", this);
    m_clearPred  = new QPushButton("Clear Predators", this);

    m_layout->addLayout(add_boids_button_layout_);
    m_layout->addWidget(m_clearAll);
    m_layout->addWidget(m_clearBoids);
    m_layout->addWidget(m_clearObs);
    m_layout->addWidget(m_clearPred);

    QObject::connect(m_clearAll, &QPushButton::clicked, this, &ButtonGroup::onClearAllPressed);
    QObject::connect(m_clearBoids, &QPushButton::clicked, this, &ButtonGroup::onClearBoidsPressed);
    QObject::connect(m_clearObs, &QPushButton::clicked, this, &ButtonGroup::onClearObsPressed);
    QObject::connect(m_clearPred, &QPushButton::clicked, this, &ButtonGroup::onClearPredPressed);
}

void ButtonGroup::onAddBoids1() { emit addBoids(10); }

void ButtonGroup::onAddBoids2() { emit addBoids(50); }

void ButtonGroup::onAddBoids3() { emit addBoids(100); }

void ButtonGroup::onClearAllPressed() {
    const std::vector<boids::BoidType> types = {boids::BOID, boids::OBSTACLE, boids::PREDATOR};
    emit                               clearBoids(types);
}

void ButtonGroup::onClearBoidsPressed() { emit clearBoids({boids::BOID}); }

void ButtonGroup::onClearObsPressed() { emit clearBoids({boids::OBSTACLE}); }

void ButtonGroup::onClearPredPressed() { emit clearBoids({boids::PREDATOR}); }

} // namespace ui
