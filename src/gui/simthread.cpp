#include "simthread.h"
#include <QMutex>

SimThread::SimThread(const std::shared_ptr<boids::Flock> flock, QObject* parent) : QThread(parent) {
    m_stop    = false;
    m_boidSim = flock;
}

SimThread::~SimThread() {
    quit();
    wait();
}

void SimThread::stopSim() {
    QMutex mutex;
    mutex.lock();
    m_stop = true;
    mutex.unlock();
}

void SimThread::run() {

    while (!m_stop) {
        m_boidSim->update();

        QList<boids::Boid> boids;
        for (const auto& [key, value] : m_boidSim->getBoids()) {
            for (const auto& v : value) {
                boids.push_back(v);
            }
        }
        emit update(boids);
        this->usleep(10000);
    }
}
