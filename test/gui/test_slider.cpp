#include "slider.h"
#include <QApplication>
#include <QSignalSpy>
#include <gtest/gtest.h>

QApplication* app = nullptr;

/**
 * @brief Test fixture that sets up a QApplication so that QObjects/QWidgets can be tested
 */
class SliderTest : public ::testing::Test {
  protected:
    virtual void SetUp() {
        if (!app) {
            int    argc = 0;
            char** argv = nullptr;
            app         = new QApplication(argc, argv);
        }
    }
};

/**
 * @brief Test the default value of the slider at construction.
 */
TEST_F(SliderTest, DefaultConstructorInitValue) {
    ui::Slider  slider("Test Slider");
    const float exp = 0.0f;
    const float res = slider.getValue();
    ASSERT_FLOAT_EQ(exp, res);
}

/**
 * @brief Test setting the value of the slider.
 */
TEST_F(SliderTest, SetValue) {
    ui::Slider  slider("Test Slider");
    const float exp = 0.1f;
    slider.setValue(exp);
    const float res = slider.getValue();
    ASSERT_FLOAT_EQ(exp, res);
}

/**
 * @brief Test that a `std::out_of_range` exception is thrown if the requested value is below the
 * allowed minimum.
 */
TEST_F(SliderTest, SetValueBelowMin) {
    ui::Slider slider("Test Slider", 0.1f, 1.0f);
    ASSERT_THROW(slider.setValue(0.0f), std::out_of_range);
}

/**
 * @brief Test that a `std::out_of_range` exception is thrown if the requested value is above the
 * allowed maximum.
 */
TEST_F(SliderTest, SetValueAboveMax) {
    ui::Slider slider("Test Slider", 0.1f, 1.0f);
    ASSERT_THROW(slider.setValue(2.0f), std::out_of_range);
}

/**
 * @brief Test that the`valueChanged()` signal is emitted when the `setValue()` method is called.
 */
TEST_F(SliderTest, ValueChangedSignal) {
    ui::Slider slider("Test Slider", 0.0f, 100.0f);
    QSignalSpy spy(&slider, &ui::Slider::valueChanged);
    slider.setValue(50.0f);
    EXPECT_EQ(spy.count(), 1);
}
