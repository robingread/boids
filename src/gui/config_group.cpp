#include "config_group.h"

namespace ui {

ConfigGroup::ConfigGroup(const QString& name, QWidget* parent) : QWidget(parent) {
    m_layout = std::make_unique<QVBoxLayout>(this);

    m_nRadSlider      = createSlider("N Radius", 0.0f, 200.0f);
    m_maxVelSlider    = createSlider("Max Vel", 0.0f, 3.0f);
    m_alignSlider     = createSlider("Align", 0.0f, 1.0f);
    m_cohesionSlider  = createSlider("Cohesion", 0.0f, 1.0f);
    m_repelSlider     = createSlider("Repel", 0.0f, 1.0f);
    m_obsRepelSlider  = createSlider("Obs Repel", 0.0f, 5.0f);
    m_predRepelSlider = createSlider("Pred Repel", 0.0f, 10.0f);
    m_repelMinDist    = createSlider("Repel Min Dist", 0.0f, 200.0f);

    m_groupBox = std::make_unique<QGroupBox>(this);
    m_groupBox->setTitle(name);
    m_groupBox->setLayout(m_layout.get());

    QVBoxLayout* l = new QVBoxLayout(this);
    l->addWidget(m_groupBox.get());
    l->geometry();
}

boids::Config ConfigGroup::getConfig() const {
    boids::Config cfg;
    cfg.neighbourhoodRadius = m_nRadSlider->getValue();
    cfg.maxVelocity         = m_maxVelSlider->getValue();
    cfg.alignmentScale      = m_alignSlider->getValue();
    cfg.coheasionScale      = m_cohesionSlider->getValue();
    cfg.repelScale          = m_repelSlider->getValue();
    cfg.obstacleRepelScale  = m_obsRepelSlider->getValue();
    cfg.predatorRepelScale  = m_predRepelSlider->getValue();
    cfg.repelMinDist        = m_repelMinDist->getValue();
    return cfg;
}

void ConfigGroup::setConfig(const boids::Config& cfg) {
    m_nRadSlider->setValue(cfg.neighbourhoodRadius);
    m_maxVelSlider->setValue(cfg.maxVelocity);
    m_alignSlider->setValue(cfg.alignmentScale);
    m_cohesionSlider->setValue(cfg.coheasionScale);
    m_repelSlider->setValue(cfg.repelScale);
    m_obsRepelSlider->setValue(cfg.obstacleRepelScale);
    m_predRepelSlider->setValue(cfg.predatorRepelScale);
    m_repelMinDist->setValue(cfg.repelMinDist);
}

std::unique_ptr<Slider> ConfigGroup::createSlider(const QString& name, const float& minValue,
                                                  const float& maxValue) {
    std::unique_ptr<Slider> ptr = std::make_unique<Slider>(name, minValue, maxValue, this);
    QObject::connect(ptr.get(), &Slider::valueChanged, this, &ConfigGroup::onSliderValueChanged);
    m_layout->addWidget(ptr.get());
    return ptr;
}

void ConfigGroup::onSliderValueChanged() { this->configChanged(); }

} // namespace ui