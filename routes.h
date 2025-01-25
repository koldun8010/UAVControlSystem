#ifndef ROUTES_H
#define ROUTES_H

#include "placemark.h"
#include "qnetworkdiskcache.h"
#include "qpushbutton.h"
#include <QWidget>
#include <QDropEvent>//

namespace Ui {
class Routes;
}

class Routes : public QWidget
{
    Q_OBJECT

public:
    explicit Routes(QWidget *parent = nullptr);
    ~Routes();    

signals:
    void loadPoint(std::vector<std::vector<double>> point);
    void reset();
    void updateOrder();
    void pointChanged(const QGV::GeoPos geoPos_start, const QGV::GeoPos geoPos_stop);

private slots:
    void on_addPointButton_clicked();
    void on_resetButton_clicked();
    void on_setRouteButton_clicked();
    void updateSectionsOrder();
    void deleteRow();
    void pressHandler(QPointF projPos);
    void addPoint(double lat, double longt, double alt, int rows);

public slots:
    void loadMap();
    void reorder(int logicalIndex, int oldVisualIndex, int newVisualIndex);

public:
    Placemark* placemark;

private:
    Ui::Routes *ui;
    std::vector<std::vector<double>> waypoints;
    QNetworkAccessManager* mManager;
    QNetworkDiskCache* mCache;
    QString cacheDir;
};

#endif // ROUTES_H
