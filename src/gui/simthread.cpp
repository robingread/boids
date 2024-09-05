#include "simthread.h"
#include <QMutex>
#include <iostream>


SimThread::SimThread(QObject *parent) : QThread(parent)
{
    m_stop = false;
}


SimThread::~SimThread()
{
    quit();
    wait();
}


void SimThread::addNewItem(const QPointF &pos)
{
    std::cout << "Running addNewItem() slot" << std::endl;

    QMutex mutex;
    mutex.lock();
    m_boidSim.addBoid(pos.x(), pos.y());
    mutex.unlock();
}


void SimThread::stopSim()
{
    QMutex mutex;
    mutex.lock();
    m_stop = true;
    mutex.unlock();
}


void SimThread::updateSceneBounds(const QRectF &bounds)
{
    QMutex mutex;
    mutex.lock();
    m_boidSim.setSceneBounds(bounds);
    mutex.unlock();
}


void SimThread::run()
{
    std::cout << "Runing the main sim loop..." << std::endl;

    while (!m_stop)
    {
        m_boidSim.update();

        QList<boids::Boid> boids;
        for (const auto &b : m_boidSim.getBoids())
        {
            boids.push_back(b);
        }
        emit update(boids);
        this->usleep(10000);
    }
}