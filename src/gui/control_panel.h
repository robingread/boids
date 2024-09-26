#pragma once

#include "button_group.h"
#include "config_group.h"
#include <QWidget>

namespace ui {

class ControlPanelWidget : public QWidget {

    Q_OBJECT

  public:
    std::unique_ptr<ButtonGroup> m_buttonGroup;
    std::unique_ptr<ConfigGroup> m_boidCfgGroup;
    std::unique_ptr<ConfigGroup> m_predatorCfgGroup;
    ControlPanelWidget(QWidget* parent = nullptr);
};

} // namespace ui