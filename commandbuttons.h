#ifndef COMMANDBUTTONS_H
#define COMMANDBUTTONS_H

#include "approvaldialog.h"
#include "notification.h"
#include "routes.h"
#include <GeographicLib/LocalCartesian.hpp>
#include <QWidget>
#include <QThread>
#include <QTimer>
#include <QTcpServer>
#include <vector>

#include <SDL2/SDL_events.h>
#include <SDL2/SDL_joystick.h>
#include <api/server.h>

#include <placemark.h>
#include <qnetworkaccessmanager.h>
#include <qnetworkdiskcache.h>
#include <indicators.h>

enum class DataType {
    Message,
    Vector
};

namespace Ui {
class CommandButtons;
}

class CommandButtons : public QWidget
{
    Q_OBJECT

public:
    explicit CommandButtons(QWidget *parent = nullptr);
    ~CommandButtons();

public:
    Placemark* placemark;

signals:
    void connected();
    void pointChanged(const QGV::GeoPos geoPos_start, const QGV::GeoPos geoPos_stop);

private:
    Ui::CommandButtons* ui;
    Indicators* indics;
    Routes* routes;
    ApprovalDialog* dialog;
    Notification* notif;
    Server* server;
    QNetworkAccessManager* mManager;
    QNetworkDiskCache* mCache;
    QString cacheDir;
    QThread* flightStickThread;
    QThread* simulationThread;
    SDL_Joystick* flightStick;
    SDL_Event event;
    std::vector<std::vector<double>> route;
    double speedAngle;
    size_t commandId;
    QTimer simulationTimer;
    QTcpServer* localServ;
    QTcpSocket *socket;
    std::vector<double> X;
    QString msg;
    GeographicLib::LocalCartesian *localCartesian;
    double x;
    double y;
    double z;
    double V;
    double theta;
    double Psi;
    double gamma;
    double nx;
    double ny;
    double nydot;
    double nz;
    int tickCount;

    void closeDialog();
    void execute();
    void closeNotification();
    void horizonIncline(int yaw);
    void flightStickHandle();
    void placemarkHandle();
    void takeoff();
    void land();
    void returnToLaunch();
    void start(quint16 port);

private slots:
    void on_connectButton_clicked();
    void on_takeOffButton_clicked();
    void on_landButton_clicked();
    void on_returnButton_clicked();
    void on_routeButton_clicked();
    void on_prepButton_clicked();
    void on_mapCamToggle_toggled(bool checked);
    void on_modeSlider_valueChanged(int value);
    void on_indicatorButton_clicked();
    void on_yawSlider_valueChanged(int value);
    void on_pitchSlider_valueChanged(int value);
    void on_speedSlider_valueChanged(int value);
    void updateRoute(std::vector<std::vector<double>> waypoints);
    void resetRoute();
    void sendVector(const std::vector<double> &data);
    void sendMessage(const QString &message);
    void onNewConnection();
    void readData();

    void on_yawSlider_sliderPressed();
    void on_pitchSlider_sliderPressed();
};

#endif // COMMANDBUTTONS_H
