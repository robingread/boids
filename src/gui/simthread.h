#ifndef SIMTHREAD_H
#define SIMTHREAD_H

#include <flock.h>
#include <QList>
#include <QPointF>
#include <QRectF>
#include <QThread>


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
class SimThread : public QThread
{
    Q_OBJECT

public:
    SimThread(QObject *parent=0);
    ~SimThread();

    /**
     * @brief Run the simulation thread.
     */
    void run() override;

private:
    bool m_stop;
    boids::Flock m_boidSim;

public slots:

    /**
     * @brief Add a new boid to the simulation at a given location.
     * 
     * @param pos Position for the new boid in the format (x, y), in a normalized 
     * coorindate space.
     */
    void addNewItem(const QPointF &pos);

    /**
     * @brief Stop the simulation thread.
     */
    void stopSim();

    /**
     * @brief Update the boundary of the simulation scene. This is to cater for when the
     * display window chnages size.
     * 
     * @param bounds 
     */
    void updateSceneBounds(const QRectF &bounds);


signals:
    void update(const QList<boids::Boid> &boids);
};

#endif // SIMTHREAD_H
