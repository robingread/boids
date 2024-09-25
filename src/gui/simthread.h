#ifndef SIMTHREAD_H
#define SIMTHREAD_H

#include <QList>
#include <QPointF>
#include <QRectF>
#include <QThread>
#include <flock.h>
#include <memory>

/**
 * @brief The SimThread class is used to run the main logic of the Boids simulation.
 *
 * This is to split away the running of the main logic/calculations and the display of
 * the simulation via the GUI.
 *
 * The class provides an interface allowing you to add new boids (e.g., with a mouse
 * click), to change the size of the display area, or stop the simulaton all together.
 *
 * The thread emits an update() signal, which can be used to trigger a GUI update.
 */
class SimThread : public QThread {
    Q_OBJECT

  public:
    std::shared_ptr<boids::Flock> m_boidSim;

    SimThread(const std::shared_ptr<boids::Flock> flock, QObject* parent = 0);
    ~SimThread();

    /**
     * @brief Run the simulation thread.
     */
    void run() override;

  private:
    bool m_stop;

  public slots:
    /**
     * @brief Stop the simulation thread.
     */
    void stopSim();

  signals:
    void update(const QList<boids::Boid>& boids);
};

#endif // SIMTHREAD_H
