#include "commandbuttons.h"
#include "pointmark.h"
#include "routeline.h"
#include "ui_commandbuttons.h"
#include <cmath>
#include <QTcpServer>
#include <GeographicLib/Geocentric.hpp>
#include <GeographicLib/LocalCartesian.hpp>

#include <SDL2/SDL.h>
#include <plugins/telemetry/telemetry.h>

#include <placemark.h>
#include <qdir.h>
#include <qnetworkaccessmanager.h>
#include <qnetworkdiskcache.h>
#include <QPixmap>
#include <QDebug>

#include <QGeoView/QGVLayerGoogle.h>
#include <QGeoView/QGVCamera.h>

CommandButtons::CommandButtons(QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::CommandButtons)
{
    ui->setupUi(this);
    routes = new Routes;
    dialog = new ApprovalDialog;
    notif = new Notification;
    indics = new Indicators;
    dialog = new ApprovalDialog;
    QObject::connect(dialog, &ApprovalDialog::closeDialog, this, &CommandButtons::closeDialog);
    QObject::connect(dialog, &ApprovalDialog::exec, this, &CommandButtons::execute);
    QObject::connect(notif, &Notification::closeNotification, this, &CommandButtons::closeNotification);
    QObject::connect(routes, &Routes::reset, this, &CommandButtons::resetRoute);
    QObject::connect(routes, &Routes::loadPoint, this, &CommandButtons::updateRoute);
    QObject::connect(this, &CommandButtons::connected, routes, &Routes::loadMap);
    connect(&simulationTimer, &QTimer::timeout, this, &CommandButtons::placemarkHandle);
    simulationTimer.setInterval(100);
    tickCount = 0;

    ui->connectionRadioButton->setEnabled(false);
    ui->pitchSlider->setEnabled(false);
    ui->yawSlider->setEnabled(false);
    ui->speedSlider->setEnabled(false);
    ui->modeSlider->setEnabled(false);
    ui->prepButton->setEnabled(false);
    ui->takeOffButton->setEnabled(false);
    ui->landButton->setEnabled(false);
    ui->returnButton->setEnabled(false);

    ui->angleScaleLabel0->lower();
    ui->angleScaleLabel1->lower();
    ui->angleScaleLabel2->lower();
    ui->angleScaleLabel3->lower();
    ui->angleScaleLabel4->lower();
    ui->angleScaleLabel5->lower();
    ui->angleScaleLabel6->lower();
    ui->horizonCompasScaleLabel->lower();   //БЛОК С LOWER - ПЕРЕНОС ШКАЛЫ КОМПАСА
    ui->horizonHorLabel->lower();           //И ЗАДНИКА АВИАГОРИЗОНТА НА НУЖНЫЙ УРОВЕНЬ

    ui->fpvLabel->setVisible(false);
    ui->fpvLabel->setStyleSheet("QLabel { background-color : gray; }");

    ui->altScaleLabel2->setText(QString::number(50));
    ui->altScaleLabel3->setText(QString::number(40));
    ui->altScaleLabel4->setText(QString::number(30));
    ui->altScaleLabel5->setText(QString::number(20));
    ui->altScaleLabel6->setText(QString::number(10));
    ui->altScaleLabel7->setText(QString::number(0));

    QPixmap joystickPointerPix;
    joystickPointerPix.load("/home/kolsun/ControlSystemGUI/images/pointerVer2.png");
    ui->joystickPointerLabel->setPixmap(joystickPointerPix);
    QPixmap horizonHorPix;
    horizonHorPix.load("/home/kolsun/ControlSystemGUI/images/horizonHor.png");
    ui->horizonHorLabel->setPixmap(horizonHorPix);
    QPixmap horizonViewPix;
    horizonViewPix.load("/home/kolsun/ControlSystemGUI/images/horizonView.png");
    ui->horizonViewLabel->setPixmap(horizonViewPix);
    QPixmap horizonMarksPix;
    horizonMarksPix.load("/home/kolsun/ControlSystemGUI/images/horizonMarks.png");
    ui->horizonMarksLabel->setPixmap(horizonMarksPix);
    QPixmap horizonFgPix;
    horizonFgPix.load("/home/kolsun/ControlSystemGUI/images/horizonPlane.png");
    ui->horizonFgLabel->setPixmap(horizonFgPix);
    QPixmap horizonAnglePix;
    horizonAnglePix.load("/home/kolsun/ControlSystemGUI/images/horizonAngle.png");
    ui->horizonAngleLabel->setPixmap(horizonAnglePix);
    QPixmap horizonCompasScalePix;
    horizonCompasScalePix.load("/home/kolsun/ControlSystemGUI/images/horizonCompasScale.png");
    ui->horizonCompasScaleLabel->setPixmap(horizonCompasScalePix);
    QPixmap horizonSpeedScalePix;
    horizonSpeedScalePix.load("/home/kolsun/ControlSystemGUI/images/horizonSpeedScale.png");
    ui->horizonSpeedScaleLabel->setPixmap(horizonSpeedScalePix);
    QPixmap horizonAltScalePix;
    horizonAltScalePix.load("/home/kolsun/ControlSystemGUI/images/horizonAltScale.png");
    ui->horizonAltScaleLabel->setPixmap(horizonAltScalePix);

    QDir("cachedir").removeRecursively();
    mCache = new QNetworkDiskCache(this);
    mCache->setCacheDirectory(cacheDir);
    mManager = new QNetworkAccessManager(this);
    mManager->setCache(mCache);
    QGV::setNetworkManager(mManager);
    QGVLayer *mapLayer = new QGVLayerGoogle(QGV::TilesType::Hybrid);
    ui->mapWidget->addItem(mapLayer);
    ui->mapWidget->select(mapLayer);
    ui->mapWidget->flyTo(QGVCameraActions(ui->mapWidget).scaleTo(
        QGV::GeoRect(QGV::GeoPos(55.6, 37.5), QGV::GeoPos(55.9, 37.8))));

    SDL_Init(SDL_INIT_JOYSTICK);
    SDL_JoystickEventState(SDL_ENABLE);
    flightStick = SDL_JoystickOpen(0);
}

CommandButtons::~CommandButtons()
{
    delete ui;
}

void CommandButtons::start(quint16 port)
{
    if (!localServ->listen(QHostAddress::Any, port)) {
        qDebug() << "Server failed to start: " << localServ->errorString();
    } else {
        qDebug() << "Server started on port: " << port;
    }
}

void CommandButtons::onNewConnection()
{
    socket = localServ->nextPendingConnection();
    connect(socket, &QTcpSocket::readyRead, this, &CommandButtons::readData);
}

void CommandButtons::sendVector(const std::vector<double> &data) {
    QByteArray vectorData;
    QDataStream out(&vectorData, QIODevice::WriteOnly);
    out.setVersion(QDataStream::Qt_6_0);

    out << DataType::Vector;
    out << (quint32)data.size();
    for (double value : data) {
        out << value;
    }
    socket->write(vectorData);
}

void CommandButtons::sendMessage(const QString &message) {
    QByteArray block;
    QDataStream out(&block, QIODevice::WriteOnly);
    out << DataType::Message;
    out << message.toUtf8();
    socket->write(block);
}

void CommandButtons::readData() {
    QByteArray data = socket->readAll();
    QDataStream in(&data, QIODevice::ReadOnly);
    in.setVersion(QDataStream::Qt_6_0);

    DataType type;
    in >> type;

    if (type == DataType::Message) {
        QByteArray messageData;
        in >> messageData;
        QString message = QString::fromUtf8(messageData);
        qDebug() << "Получено сообщение:" << message;
        msg = message;

        if (msg == "Hello from client!") {
            //ui->prepButton->setEnabled(true);                     //Проверка на подключение клиента
        }
    } else if (type == DataType::Vector) {
        quint32 size;
        in >> size;
        std::vector<double> receivedData(size);
        for (int i = 0; i < size; ++i) {
            in >> receivedData[i];
        }
        qDebug() << "Полученный вектор:";
        for (double value : receivedData) {
            //qDebug() << value;
        }
        X = receivedData;
    } else {
        qDebug() << "Неизвестный тип данных";
    }
}

void CommandButtons::on_connectButton_clicked()
{    
    this->server = new Server("udp://:14540");

    ui->mapWidget->flyTo(QGVCameraActions(ui->mapWidget).scaleTo(
        QGV::GeoRect(QGV::GeoPos(55.76, 37.68), QGV::GeoPos(55.77, 37.69))                      //ХАРДКОД - ИЗМЕНИТЬ К ДИПЛОМУ
        //BMSTU Main Building - GeoPos: 55.76618001715375, 37.6850103977006
        ));
    placemark = new Placemark(QGV::GeoPos(55.76618001715375, 37.6850103977006));
    placemark->setLat(55.76618001715375);
    placemark->setLongt(37.6850103977006);
    /*
    QTransform transform;
    transform = this->placemark->effectiveTransform().rotate(90);
    this->placemark->loadImage(this->placemark->getImage().transformed(transform));             //Повернуть плэйсмарк
*/
    double lat0 = 55.76618001715375;
    double longt0 = 37.6850103977006;
    double h0 = 0;
    localCartesian = new GeographicLib::LocalCartesian(lat0, longt0, h0);

    for (int i = 0; i < 11; i++) {
        X.push_back(1);
    }

    x = 1;
    y = 1;
    z = 1;
    V = 1;
    theta = 1;
    Psi = 1;
    gamma = 1;
    nx = 1;
    ny = 1;
    nydot = 1;
    nz = 1;
/*
    server->setLatitude(55.76618001715375);
    server->setLongitude(37.6850103977006);
    server->setAltitude(0);
*/
    ui->mapWidget->addItem(this->placemark);

    ui->connectionRadioButton->setEnabled(true);
    ui->connectionRadioButton->click();
    ui->connectionRadioButton->setEnabled(false);
    ui->connectButton->setEnabled(false);
    ui->prepButton->setEnabled(true);

    QPixmap fpvPix;
    fpvPix.load("/home/kolsun/ControlSystemGUI/images/fpv_screenshot.png");
    ui->fpvLabel->setPixmap(fpvPix);

    ui->battery1Label->setStyleSheet("QLabel { border-style: outset; border-width: 1px; background-color : yellow }");
    ui->battery2Label->setStyleSheet("QLabel { border-style: outset; border-width: 1px; background-color : yellow }");
    ui->battery3Label->setStyleSheet("QLabel { border-style: outset; border-width: 1px; background-color : yellow }");
    ui->battery4Label->setStyleSheet("QLabel { border-style: outset; border-width: 1px; background-color : yellow }");

    localServ = new QTcpServer(this);
    socket = new QTcpSocket(this);
    connect(localServ, &QTcpServer::newConnection, this, &CommandButtons::onNewConnection);
    start(12345);
}

void CommandButtons::on_takeOffButton_clicked()
{
    commandId = 1;
    dialog->show();
}


void CommandButtons::on_landButton_clicked()
{
    commandId = 2;
    dialog->show();
}


void CommandButtons::on_returnButton_clicked()
{
    commandId = 3;
    dialog->show();
}

void CommandButtons::on_routeButton_clicked()
{
    routes->show();
}

void CommandButtons::on_prepButton_clicked()
{
    ui->takeOffButton->setEnabled(true);
    ui->landButton->setEnabled(true);
    ui->returnButton->setEnabled(true);

    ui->modeSlider->setEnabled(true);
    ui->prepButton->setEnabled(false);

    ui->modeSlider->setValue(3);
    ui->modeSlider->setValue(1);

    indics->ready();
    indics->show();
    notif->checked(true);                                                                   //НЕТ РЕАЛЬНОЙ ПРОВЕРКИ - ХАРДКОД, ПОФИКСИТЬ К ДИПЛОМУ
    notif->show();

    emit(connected());
    speedAngle = 0.0;
    commandId = 0;
}

void CommandButtons::on_mapCamToggle_toggled(bool checked)
{
    if (checked) {
        ui->fpvLabel->setVisible(true);
        ui->mapWidget->setVisible(false);
    } else {
        ui->mapWidget->setVisible(true);
        ui->fpvLabel->setVisible(false);
    }
}


void CommandButtons::on_modeSlider_valueChanged(int value)
{
    if (value == 1) {
        if (flightStickThread->isRunning()) {
            flightStickThread->requestInterruption();
            //qDebug() << "STOPPED";
        }

        ui->pitchSlider->setEnabled(true);
        ui->yawSlider->setEnabled(true);
        ui->speedSlider->setEnabled(true);
        ui->pitchSlider->setValue(0);

        if (socket != nullptr && socket->state() == QTcpSocket::ConnectedState) {
            sendMessage(QString::number(1));
        }

    } else if (value == 2) {
        if (flightStickThread->isRunning()) {
            flightStickThread->requestInterruption();
            //qDebug() << "STOPPED";
        }

        if (!simulationTimer.isActive()) {
            simulationTimer.start();
        }

        ui->pitchSlider->setEnabled(false);
        ui->yawSlider->setEnabled(false);
        ui->speedSlider->setEnabled(false);

        if (socket != nullptr && socket->state() == QTcpSocket::ConnectedState) {
            ui->yawSlider->setValue(0);
            ui->pitchSlider->setValue(0);
            Psi = 0;
            theta = 0;
            if (!route.empty()) {
                double zDes, xDes, yDes, PsiDes;
                localCartesian->Forward(route[0][0], route[0][1], route[0][2], zDes, xDes, yDes);
                PsiDes = acos((xDes - x) / sqrt((xDes - x) * (xDes - x) + (z - zDes) * (z - zDes)));
                if (atan((z - zDes) / (xDes - x)) < 0) {
                    PsiDes *= -1;
                }
                if (abs(PsiDes) < M_PI / 2) {
                    PsiDes *= -1;
                }
                sendVector({x, y, z, V, theta, Psi, gamma, nx, ny, nydot, nz, 35, route[0][2], PsiDes});
            } else {
                sendVector({x, y, z, V, theta, Psi, gamma, nx, ny, nydot, nz, 35, 50, 0});
            }
        }

    } else if (value == 3) {
        flightStickThread = QThread::create([this]{
            while (1) {
                if (QThread::currentThread()->isInterruptionRequested()) {
                    return;
                }
                flightStickHandle();
            }
        });

        ui->pitchSlider->setEnabled(true);
        ui->yawSlider->setEnabled(true);
        ui->speedSlider->setEnabled(true);

        if (socket != nullptr && socket->state() == QTcpSocket::ConnectedState) {
            sendMessage(QString::number(3));
        }

        flightStickThread->start();
    }
}


void CommandButtons::on_indicatorButton_clicked()
{
    indics->show();
}

void CommandButtons::closeDialog()
{
    dialog->close();
}

void CommandButtons::closeNotification()
{
    notif->close();
}

void CommandButtons::resetRoute()
{
    route.clear();
}

void CommandButtons::execute()
{
    if (commandId == 1) {
        takeoff();
    } else if (commandId == 2) {
        land();
    } else if (commandId == 3) {
        returnToLaunch();
    }
    commandId = 0;
}

void CommandButtons::updateRoute(std::vector<std::vector<double>> waypoints)
{
    route = waypoints;
    for (int i = 0; i < waypoints.size(); i++) {
        qDebug() << waypoints[i];

        double prevLat = placemark->getLat();
        double prevLongt = placemark->getLongt();
        double lat = waypoints[i][0];
        double longt = waypoints[i][1];

        if (i > 0) {
            prevLat = waypoints[i - 1][0];
            prevLongt = waypoints[i - 1][1];
        }

        qDebug() << prevLat << prevLongt << lat << longt;
        QPointF projStart = ui->mapWidget->getProjection()->geoToProj(QGV::GeoPos(prevLat, prevLongt));
        QPointF projStop = ui->mapWidget->getProjection()->geoToProj(QGV::GeoPos(lat, longt));

        RouteLine *routeLine = new RouteLine(projStart, projStop, QColor(Qt::blue));
        ui->mapWidget->addItem(routeLine);
        Pointmark *point = new Pointmark(QGV::GeoPos(lat, longt));
        ui->mapWidget->addItem(point);

        if (i == 0) {
            QObject::connect(this, &CommandButtons::pointChanged, routeLine, &RouteLine::updatePoints);
        }
    }

    double zDes, xDes, yDes, PsiDes;
    localCartesian->Forward(route[0][0], route[0][1], route[0][2], zDes, xDes, yDes);
    PsiDes = acos((xDes - x) / sqrt((xDes - x) * (xDes - x) + (z - zDes) * (z - zDes)));
    if (atan((z - zDes) / (xDes - x)) < 0) {
        PsiDes *= -1;
    }
    if (abs(PsiDes) < M_PI / 2) {
        PsiDes *= -1;
    }

    qDebug() << route[0][0] << route[0][1] << route[0][2] << zDes << xDes << yDes << z << x << y;
    if (socket != nullptr&& socket->state() == QTcpSocket::ConnectedState) {
        sendVector({35, route[0][2], PsiDes});
    }
}

void CommandButtons::horizonIncline(int roll) {
    QTransform transform;
    transform.rotate(roll);

    QPixmap horizonFgPix;
    horizonFgPix.load("/home/kolsun/ControlSystemGUI/images/horizonPlane.png");
    QPixmap horizonFgPix_new = horizonFgPix.transformed(transform);
    ui->horizonFgLabel->setPixmap(horizonFgPix_new);
}


void CommandButtons::on_yawSlider_valueChanged(int value)
{
    ui->yawValueLabel->setText(QString::number(value) + "°");
    //horizonIncline(value);
}

void CommandButtons::on_pitchSlider_valueChanged(int value)
{
    ui->pitchValueLabel->setText(QString::number(value) + "°");
    ui->horizonHorLabel->move(-5, -285 + value * 4);
}

void CommandButtons::on_speedSlider_valueChanged(int value)
{
    ui->speedValueLabel->setText(QString::number(value) + "км/ч");
    ui->absSpeedValLabel->setText(QString::number(value));
    ui->horizonSpeedScaleLabel->move(-88, -1065 + value * 6.8);
}

void CommandButtons::flightStickHandle() {
    unsigned int j = 0;
    for (j = 0; j < 30000000; j++);
    SDL_PollEvent(&event);
/*
    qDebug() << "roll: " << (SDL_JoystickGetAxis(flightStick, 0) + 129) / 182.0;
    qDebug() << "pitch: " << (SDL_JoystickGetAxis(flightStick, 1) + 129); // / 364.083;
    qDebug() << "speed: " << -(SDL_JoystickGetAxis(flightStick, 2) - 32768) / 436.9;
    qDebug() << "yaw: " << 180 + (SDL_JoystickGetAxis(flightStick, 3) + 129) / 182.1;
*/
/*
    server->setRollDegree((SDL_JoystickGetAxis(flightStick, 0) + 129) / 182.0);
    //server->setPitchDegree(180 + (SDL_JoystickGetAxis(flightStick, 3) + 129) / 182.1);
    server->setVelocity(-(SDL_JoystickGetAxis(flightStick, 2) - 32768) / 436.9);
    server->setYawDegree(180 + (SDL_JoystickGetAxis(flightStick, 3) + 129) / 182.1);
*/
    horizonIncline(trunc(SDL_JoystickGetAxis(flightStick, 0) + 129) / 362.077);
    //ui->pitchSlider->setSliderPosition((SDL_JoystickGetAxis(flightStick, 1) + 129) / 364.083);
    ui->speedSlider->setSliderPosition(-(SDL_JoystickGetAxis(flightStick, 2) - 32768) / 436.9);
    ui->yawSlider->setSliderPosition((SDL_JoystickGetAxis(flightStick, 3) + 129) / 182.1);

    ui->joystickCursorLabel->move(880 + (SDL_JoystickGetAxis(flightStick, 0) + 129) / 500.3,
                                  ui->joystickCursorLabel->y());
    //ui->joystickCursorLabel->move(ui->joystickCursorLabel->x(), 640 + (SDL_JoystickGetAxis(flightStick, 1) + 129) / 500.3);
}

void CommandButtons::placemarkHandle() {
    tickCount++;
    double lat, longt, alt;

    if (ui->modeSlider->value() == 2) {
        x = X[0];
        y = X[1];
        z = X[2];
        V = X[3];
        theta = X[4];
        Psi = X[5];
        gamma = X[6];
        nx = X[7];
        ny = X[8];
        nydot = X[9];
        nz = X[10];

        localCartesian->Reverse(-z, x, y, lat, longt, alt);
        speedAngle = X[5];
        ui->speedSlider->setValue(V * 3.6);
        ui->pitchSlider->setValue(theta * 180 / M_PI);
        horizonIncline(-gamma * 180 / M_PI);

        if ((!route.empty()) && (abs(longt - route[0][1]) < 0.0005) && (abs(route[0][0] - lat)) < 0.0005) {
            qDebug() << "point reached: " << route[0];
            route.erase(route.begin());
        }
    } else {
        speedAngle += double(ui->yawSlider->value()) / 180.0 * M_PI / 180;
        if (abs(speedAngle * 180 / M_PI) > 180) {
            speedAngle *= -1;
        }

        V = ui->speedSlider->value() / 3.6;
        theta = ui->pitchSlider->value() * M_PI / 180;
        Psi = ui->yawSlider->value() * M_PI / 180;

        x += 0.1 * V * cos(theta) * cos(speedAngle);
        y += 0.1 * V * sin(theta);
        z -= 0.1 * V * cos(theta) * sin(speedAngle);

        localCartesian->Reverse(-z, x, y, lat, longt, alt);
        nx = 1;
        ny = 1;
        nydot = 1;
        nz = 1;

/*
arctan( V^2/g*R),
где V – скорость самолета ( в м/с),
g- ускорение земного притяжения ( 9,81 м/с2),
R- радиус поворота (в м).
Подробнее на: https://avia.pro/blog/kren-samoleta
*/
        if (ui->modeSlider->value() == 1) {
            //gamma = atan(V * V * asin(Psi / 2 * 180 / M_PI) / 9.81 / 100);
            horizonIncline(ui->yawSlider->value() /  3);
        }   //ЧТОБЫ БЫЛО, ХАРДКОД, ИБО ЗАВИСИМОСТИ НИКАКОЙ НЕТ
    }

    if (y < 0) {
        y = abs(y);
    }
    ui->altValLabel->setText(QString::number(int(y)));
    if (y < 30) {
        ui->horizonAltScaleLabel->move(432, -620 + int(y) % 30 * 6.8);
        ui->altScaleLabel0->move(425, -296 + int(y) % 30 * 6.8);
        ui->altScaleLabel1->move(425, -228 + int(y) % 30 * 6.8);
        ui->altScaleLabel2->move(425, -160 + int(y) % 30 * 6.8);
        ui->altScaleLabel3->move(425, -92 + int(y) % 30 * 6.8);
        ui->altScaleLabel4->move(425, -24 + int(y) % 30 * 6.8);
        ui->altScaleLabel5->move(425, 44 + int(y) % 30 * 6.8);
        ui->altScaleLabel6->move(425, 112 + int(y) % 30 * 6.8);
        ui->altScaleLabel7->move(425, 180 + int(y) % 30 * 6.8);
    } else {
        ui->horizonAltScaleLabel->move(432, -484 + int(y) % 30 * 6.8);
        ui->altScaleLabel0->setText(QString::number(trunc(y / 30) * 30 + 50));
        ui->altScaleLabel1->setText(QString::number(trunc(y / 30) * 30 + 40));
        ui->altScaleLabel2->setText(QString::number(trunc(y / 30) * 30 + 30));
        ui->altScaleLabel3->setText(QString::number(trunc(y / 30) * 30 + 20));
        ui->altScaleLabel4->setText(QString::number(trunc(y / 30) * 30 + 10));
        ui->altScaleLabel5->setText(QString::number(trunc(y / 30) * 30 - 0));
        ui->altScaleLabel6->setText(QString::number(trunc(y / 30) * 30 - 10));
        ui->altScaleLabel7->setText(QString::number(trunc(y / 30) * 30 - 20));
        ui->altScaleLabel0->move(425, -160 + int(y) % 30 * 6.8);
        ui->altScaleLabel1->move(425, -92 + int(y) % 30 * 6.8);
        ui->altScaleLabel2->move(425, -24 + int(y) % 30 * 6.8);
        ui->altScaleLabel3->move(425, 44 + int(y) % 30 * 6.8);
        ui->altScaleLabel4->move(425, 112 + int(y) % 30 * 6.8);
        ui->altScaleLabel5->move(425, 180 + int(y) % 30 * 6.8);
        ui->altScaleLabel6->move(425, 248 + int(y) % 30 * 6.8);
        ui->altScaleLabel7->move(425, 316 + int(y) % 30 * 6.8);
    }


    double compassAngle = speedAngle * 180 / M_PI;
    if (compassAngle < 0) {
        compassAngle = 360 + compassAngle;
    }
    ui->angleValueLabel->setText(QString::number(int(compassAngle)) + "°");
    ui->horizonCompasScaleLabel->move(-520 - int(compassAngle) % 20 * 6.8, 340);
    ui->angleScaleLabel0->move(25 - int(compassAngle) % 20 * 6.8, 340);
    ui->angleScaleLabel1->move(93 - int(compassAngle) % 20 * 6.8, 340);
    ui->angleScaleLabel2->move(161 - int(compassAngle) % 20 * 6.8, 340);
    ui->angleScaleLabel3->move(229 - int(compassAngle) % 20 * 6.8, 340);
    ui->angleScaleLabel4->move(297 - int(compassAngle) % 20 * 6.8, 340);
    ui->angleScaleLabel5->move(365 - int(compassAngle) % 20 * 6.8, 340);
    ui->angleScaleLabel6->move(433 - int(compassAngle) % 20 * 6.8, 340);
    ui->angleScaleLabel0->setText(QString::number(int(trunc((compassAngle + 360) / 20) * 20 - 30) % 360));
    ui->angleScaleLabel1->setText(QString::number(int(trunc((compassAngle + 360) / 20) * 20 - 20) % 360));
    ui->angleScaleLabel2->setText(QString::number(int(trunc((compassAngle + 360) / 20) * 20 - 10) % 360));
    ui->angleScaleLabel3->setText(QString::number(int(trunc(compassAngle / 20) * 20 - 0) % 360));
    ui->angleScaleLabel4->setText(QString::number(int(trunc(compassAngle / 20) * 20 + 10) % 360));
    ui->angleScaleLabel5->setText(QString::number(int(trunc(compassAngle / 20) * 20 + 20) % 360));
    ui->angleScaleLabel6->setText(QString::number(int(trunc(compassAngle / 20) * 20 + 30) % 360));

    if (tickCount % 10 == 0) {
        ui->mapWidget->flyTo(QGVCameraActions(ui->mapWidget).scaleTo(
            QGV::GeoRect(QGV::GeoPos(round(lat * 100 - 1) / 100, round(longt * 100 - 1) / 100),
                         QGV::GeoPos(round(lat * 100 + 1) / 100, round(longt * 100 + 1) / 100))));
    }

    placemark->setGeometry(QGV::GeoPos(lat, longt), QSize(64, 64), QPoint(16, 32));
    placemark->setLat(lat);
    placemark->setLongt(longt);
    routes->placemark->setGeometry(QGV::GeoPos(lat, longt), QSize(64, 64), QPoint(16, 32));
    routes->placemark->setLat(lat);
    routes->placemark->setLongt(longt);
    if (route.size() > 0) {
        emit(pointChanged(QGV::GeoPos(lat, longt), QGV::GeoPos(route[0][0], route[0][1])));
        //emit(routes->pointChanged(QGV::GeoPos(lat, longt), QGV::GeoPos(route[0][0], route[0][1])));
    }
}

void CommandButtons::takeoff() {
    //this->server->takeOff();                                                                                                  //ПОКА ЧТО СИСТЕМА НЕ ПОДКЛЮЧЕНА

    ui->modeSlider->setValue(2);
    //server->setAltitude(10);
/*    while (ui->speedSlider->value() < 120) {
        ui->speedSlider->setValue(ui->speedSlider->value() + 1);
    }*/
}

void CommandButtons::land() {
    //this->server->land();                                                                                                     //ПОКА ЧТО СИСТЕМА НЕ ПОДКЛЮЧЕНА

    ui->modeSlider->setValue(2);
    //server->setAltitude(0);
    /*
    while (ui->speedSlider->value() > 0) {
        ui->speedSlider->setValue(ui->speedSlider->value() - 1);
    }
*/
    if (socket != nullptr && socket->state() == QTcpSocket::ConnectedState) {
        sendMessage(QString::number(0));
    }
}

void CommandButtons::returnToLaunch() {
    //this->server->returnToLaunch();                                                                                           //ПОКА ЧТО СИСТЕМА НЕ ПОДКЛЮЧЕНА

    ui->modeSlider->setValue(2);
    resetRoute();
    updateRoute({{55.76618001715375, 37.6850103977006}});
}

void CommandButtons::on_yawSlider_sliderPressed()
{
    ui->yawSlider->setValue(0);
}


void CommandButtons::on_pitchSlider_sliderPressed()
{
    ui->pitchSlider->setValue(0);
}

