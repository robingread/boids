#pragma once

#include "simthread.h"
#include <QDialog>

QT_BEGIN_NAMESPACE
namespace Ui {
class Dialog;
}
QT_END_NAMESPACE

class Dialog : public QDialog {
    Q_OBJECT

  public:
    Dialog(QWidget* parent = nullptr);
    ~Dialog();

    void run();

  private:
    Ui::Dialog* ui;
    SimThread   m_sim;
};
