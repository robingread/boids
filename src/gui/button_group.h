#pragma once

#include "boids.h"
#include <QPushButton>
#include <QVBoxLayout>
#include <QWidget>

namespace ui {

/**
 * @class ButtonGroup
 * @brief A custom QWidget that provides buttons for clearing various types of items.
 *
 * This widget contains buttons to clear all items, boids, obstacles, and predators.
 * It emits signals when these buttons are pressed, allowing other parts of the system
 * to respond to these actions.
 */
class ButtonGroup : public QWidget {
    Q_OBJECT

  public:
    /**
     * @brief Constructs a ButtonGroup widget.
     *
     * Initializes the layout and buttons. Each button is responsible for triggering
     * different clearing actions.
     *
     * @param parent The parent widget. Defaults to `nullptr` if no parent is provided.
     */
    ButtonGroup(QWidget* parent = nullptr);

  private:
    QVBoxLayout* m_layout;
    QPushButton* m_clearAll;
    QPushButton* m_clearBoids;
    QPushButton* m_clearObs;
    QPushButton* m_clearPred;

  signals:
    /**
     * @brief Signal emitted when the "Clear Boids" button is pressed.
     *
     * This signal notifies connected components that specific types of boids should be cleared.
     *
     * @param types A vector of boids::BoidType representing the types of boids to be cleared.
     */
    void clearBoids(const std::vector<boids::BoidType>& types);

  private slots:
    /**
     * @brief Slot triggered when the "Clear All" button is pressed.
     *
     * This slot handles the action of clearing all items (boids, obstacles, and predators).
     */
    void onClearAllPressed();

    /**
     * @brief Slot triggered when the "Clear Boids" button is pressed.
     *
     * This slot handles the action of clearing boids and emits the `clearBoids` signal.
     */
    void onClearBoidsPressed();

    /**
     * @brief Slot triggered when the "Clear Obstacles" button is pressed.
     *
     * This slot handles the action of clearing obstacles.
     */
    void onClearObsPressed();

    /**
     * @brief Slot triggered when the "Clear Predators" button is pressed.
     *
     * This slot handles the action of clearing predators.
     */
    void onClearPredPressed();
};

} // namespace ui
