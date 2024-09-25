#pragma once

#include "config.h"
#include "slider.h"
#include <QGroupBox>
#include <QString>
#include <QVBoxLayout>
#include <QWidget>

namespace ui {

/**
 * @brief The ConfigGroup class is a GUI related clas created for conveniance. It creates a
 * QGroupBox with a vertical layout containing Sliders that manage individual parts of a
 * boids::Config object.
 *
 * All the sliders are stored internally, and the current configuration can be retreived by calling
 * the `getConfig()` method.
 */
class ConfigGroup : public QWidget {

    Q_OBJECT

  public:
    ConfigGroup(const QString& name, QWidget* parent = nullptr);

    /**
     * @brief Get the overall configuration as a boids::Config object.
     * @return boids::Config instance.
     */
    boids::Config getConfig() const;

    /**
     * @brief Set the overall configuration to display on the GUI.
     * @param cfg Configuration object.
     */
    void setConfig(const boids::Config& cfg);

  private:
    std::unique_ptr<QGroupBox>   m_groupBox;
    std::unique_ptr<QVBoxLayout> m_layout;

    std::unique_ptr<Slider> m_nRadSlider;
    std::unique_ptr<Slider> m_maxVelSlider;
    std::unique_ptr<Slider> m_alignSlider;
    std::unique_ptr<Slider> m_cohesionSlider;
    std::unique_ptr<Slider> m_repelSlider;
    std::unique_ptr<Slider> m_obsRepelSlider;
    std::unique_ptr<Slider> m_predRepelSlider;
    std::unique_ptr<Slider> m_repelMinDist;

    /**
     * @brief Create a Slider object, connect it to the onSliderValueChanged() callback, and add it
     * to the Vertical Layout.
     * @param name Name/title to display for the Slider label.
     * @return Unique pointer to the Slider.
     */
    std::unique_ptr<Slider> createSlider(const QString& name, const float& minValue,
                                         const float& maxValue);

  signals:
    /**
     * @brief Signal emitted when there has been a change to any of the sliders in this object.
     */
    void configChanged();

  private slots:
    /**
     * @brief Callback method/slot that emits a configChanged() signal.
     */
    void onSliderValueChanged();
};
} // namespace ui