#ifndef INDICATORS_H
#define INDICATORS_H

#include <QWidget>

namespace Ui {
class Indicators;
}

class Indicators : public QWidget
{
    Q_OBJECT

public:
    explicit Indicators(QWidget *parent = nullptr);
    ~Indicators();

private:
    Ui::Indicators *ui;

public slots:

    void ready();
};

#endif // INDICATORS_H
