#include "slider.h"
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QSlider>
#include <QString>
#include <QVBoxLayout>

namespace ui {

Slider::Slider(const QString& name, QWidget* parent) : Slider(name, 0.0f, 1.0f, parent) {}

Slider::Slider(const QString& name, const float& minValue, const float& maxValue, QWidget* parent)
    : QWidget(parent) {
    m_minValue = minValue;
    m_maxValue = maxValue;

    m_slider = new QSlider(this);
    m_slider->setMinimum(0);
    m_slider->setMaximum(100);
    m_slider->setMinimumWidth(100);
    m_slider->setOrientation(Qt::Horizontal);

    m_textLabel = new QLabel(this);
    m_textLabel->setText(QString(name + ":"));
    m_textLabel->setMinimumWidth(40);

    m_valueLabel = new QLabel(this);
    m_valueLabel->setText(QString::number(m_slider->value()));
    m_valueLabel->setMinimumWidth(30);

    QHBoxLayout* layout = new QHBoxLayout(this);
    layout->setMargin(0);
    layout->addWidget(m_textLabel);
    layout->addWidget(m_slider);
    layout->addWidget(m_valueLabel);

    QObject::connect(m_slider, &QSlider::valueChanged, this, &Slider::onSliderValueChanged);
    QObject::connect(m_slider, &QSlider::sliderReleased, this, &Slider::onSliderReleased);
}

float Slider::getValue() const {
    const float range = m_maxValue - m_minValue;
    const float ret   = m_minValue + (range * (m_slider->value() / 100.0f));
    return ret;
}

void Slider::setValue(const float& value) {
    if (value < m_minValue)
        throw std::out_of_range("Requested value below minimum");
    else if (value > m_maxValue)
        throw std::out_of_range("Requested value above maximum");

    const float diff  = value - m_minValue;
    const float range = m_maxValue - m_minValue;
    const int   val   = (diff / range) * 100.0f;

    m_slider->setValue(val);
    this->onSliderValueChanged();
    this->onSliderReleased();
}

void Slider::onSliderValueChanged() {
    const float value = getValue();
    m_valueLabel->setText(QString::number(value));
}
void Slider::onSliderReleased() { emit valueChanged(); }

} // namespace ui