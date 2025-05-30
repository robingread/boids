#include "dialog.h"
#include "displaygraphicsview.h"
#include <boids.h>
#include <random>

/**
 * @brief Generate a random float value within a given range using a uniform distribution.
 * @param min_value The minimum value of the range.
 * @param max_value The maximum value of the range.
 * @return Random value.
 */
float generateRandomNumber(const float min_value, const float max_value) {
    std::random_device             rd;
    std::mt19937                   gen(rd());
    std::uniform_real_distribution distr(min_value, max_value);
    return distr(gen);
}

Dialog::Dialog(QWidget* parent) : QMainWindow(parent) {

    m_flock = std::make_shared<boids::Flock>();
    m_sim   = new SimThread(m_flock, this);

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
    m_flock->setSceneBounds(rect);
}

void Dialog::onConfigChanged() {
    const auto boidCfg = m_control->m_boidCfgGroup->getConfig();
    const auto predCfg = m_control->m_predatorCfgGroup->getConfig();
    m_flock->setConfig(boidCfg, boids::BOID);
    m_flock->setConfig(predCfg, boids::PREDATOR);
}

void Dialog::addBoids(const std::size_t count) {
    const QRectF rect  = m_graphicsView->mapToScene(m_graphicsView->rect()).boundingRect();
    const float  min_x = rect.left();
    const float  max_x = rect.right();
    const float  min_y = rect.top();
    const float  max_y = rect.bottom();

    for (std::size_t i = 0; i < count; ++i) {
        const float x = generateRandomNumber(min_x, max_x);
        const float y = generateRandomNumber(min_y, max_y);
        createBoid(QPointF(x, y), boids::BoidType::BOID);
    }
}

void Dialog::createBoid(const QPointF& pos, const boids::BoidType& type) {
    m_flock->addBoid(pos.x(), pos.y(), type);
}

void Dialog::clearBoids(const std::vector<boids::BoidType>& types) {
    const auto boids = m_flock->getBoids();

    for (const auto& t : types) {
        m_graphicsView->clearBoids(boids.at(t));
        m_flock->clearBoids(t);
    }
}

void Dialog::run() {
    m_control->m_boidCfgGroup->setConfig(m_flock->getConfig(boids::BOID));
    m_control->m_predatorCfgGroup->setConfig(m_flock->getConfig(boids::PREDATOR));

    QObject::connect(m_graphicsView, &ui::DisplayGraphicsView::createItem, this,
                     &Dialog::createBoid);

    QObject::connect(m_sim, &SimThread::update, m_graphicsView,
                     &ui::DisplayGraphicsView::renderBoids);

    QObject::connect(m_control->m_boidCfgGroup.get(), &ui::ConfigGroup::configChanged, this,
                     &Dialog::onConfigChanged);

    QObject::connect(m_control->m_predatorCfgGroup.get(), &ui::ConfigGroup::configChanged, this,
                     &Dialog::onConfigChanged);

    QObject::connect(m_control->m_buttonGroup.get(), &ui::ButtonGroup::clearBoids, this,
                     &Dialog::clearBoids);

    QObject::connect(m_control->m_buttonGroup.get(), &ui::ButtonGroup::addBoids, this,
                     &Dialog::addBoids);

    const QRectF rect = m_graphicsView->mapToScene(m_graphicsView->rect()).boundingRect();
    m_flock->setSceneBounds(rect);

    m_sim->start();
}
