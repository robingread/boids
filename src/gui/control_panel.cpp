#include "control_panel.h"
#include "config_group.h"

#include <QVBoxLayout>

namespace ui {

ControlPanelWidget::ControlPanelWidget(QWidget* parent) : QWidget(parent) {
    m_buttonGroup      = std::make_unique<ButtonGroup>(this);
    m_boidCfgGroup     = std::make_unique<ConfigGroup>("Boid Config", this);
    m_predatorCfgGroup = std::make_unique<ConfigGroup>("Predator Config", this);

    QVBoxLayout* layout = new QVBoxLayout(this);
    layout->addWidget(m_buttonGroup.get());
    layout->addWidget(m_boidCfgGroup.get());
    layout->addWidget(m_predatorCfgGroup.get());
    layout->addStretch();
}
} // namespace ui