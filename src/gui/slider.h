#pragma once

#include <QLabel>
#include <QSlider>
#include <QString>
#include <QWidget>

namespace ui {

/**
 * @class Slider
 * @brief A custom widget combining a QLabel and a QSlider for user input.
 *
 * The Slider class is a composite widget designed for displaying and controlling
 * a floating-point value within a specified range. It provides a textual label
 * (`m_textLabel`) and a numeric label (`m_valueLabel`) showing the current value of the slider.
 * The slider internally uses a `QSlider` to control values between 0 and 100,
 * but the class automatically maps this to a user-defined range.
 *
 * The class offers:
 * - Two constructors for initialization with or without specifying minimum and maximum values.
 * - Methods to retrieve and set the slider's value within the allowed range.
 * - An event to notify changes in the slider's value (`valueChanged()` signal).
 *
 * @note The widget throws a `std::out_of_range` exception if an attempt is made to set
 * a value outside the range specified by `m_minValue` and `m_maxValue`.
 */
class Slider : public QWidget {
    Q_OBJECT
  public:
    /**
     * @brief Construct a new Slider object
     * @param name
     * @param parent
     */
    explicit Slider(const QString& name, QWidget* parent = nullptr);

    /**
     * @brief Construct a new Slider object
     * @param name
     * @param minVlaue
     * @param maxValue
     * @param parent
     */
    explicit Slider(const QString& name, const float& minVlaue, const float& maxValue,
                    QWidget* parent = nullptr);

    /**
     * @brief Get the value of the slider.
     *
     * This will read the raw value of the QSlider (in the range [0, 100]) and the apply it
     * to fall within the value set by `m_minValue` and `m_maxValue`.
     *
     * @return Slider value.
     */
    float getValue() const;

    /**
     * @brief Sets the slider value within the allowed range.
     *
     * This function sets the value of the slider, adjusting the internal QSlider
     * to reflect the specified value. If the requested value is outside the
     * allowed range (determined by `m_minValue` and `m_maxValue`), an exception
     * will be thrown.
     *
     * @param value The requested value to set the slider to. This should be
     *              between `m_minValue` and `m_maxValue`.
     *
     * @throws std::out_of_range If the requested value is less than `m_minValue`
     *                           or greater than `m_maxValue`.
     */
    void setValue(const float& value);

  private:
    float    m_minValue;
    float    m_maxValue;
    QSlider* m_slider;
    QLabel*  m_valueLabel;
    QLabel*  m_textLabel;

  private slots:
    void onSliderValueChanged();
    void onSliderReleased();

  signals:
    void valueChanged();
};

} // namespace ui
