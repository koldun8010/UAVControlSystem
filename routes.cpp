#include "routes.h"
#include "QGeoView/QGVLayer.h"
#include "QGeoView/QGVLayerGoogle.h"
#include "commandbuttons.h"
#include "pointmark.h"
#include "qdir.h"
#include "qnetworkdiskcache.h"
#include "routeline.h"
#include "ui_routes.h"

Routes::Routes(QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::Routes)
{
    ui->setupUi(this);
    ui->routesTableWidget->verticalHeader()->setSectionsMovable(true);
    //ui->routesTableWidget->setColumnWidth(3, 5);
    ui->routesTableWidget->horizontalHeader()->resizeSection(3, 27);
    /*
    ui->sTableWidget->setSelectionMode(QAbstractItemView::ExtendedSelection);
    ui->routesTableWidget->setSelectionBehavior(QAbstractItemView::SelectRows);
    ui->routesTableWidget->setDragDropMode(QAbstractItemView::InternalMove);
    ui->routesTableWidget->setDragEnabled(true);
    ui->routesTableWidget->setAcceptDrops(true);
    ui->routesTableWidget->setDropIndicatorShown(true);
    ui->routesTableWidget->setDragDropOverwriteMode(false);
*/
    QObject::connect(ui->routesTableWidget->verticalHeader(), &QHeaderView::sectionMoved, this, &Routes::reorder);
}

Routes::~Routes()
{
    delete ui;
}

void Routes::on_addPointButton_clicked()
{
    double lat = ui->latitudeSpinBox->value();
    double longt = ui->longitudeSpinBox->value();
    double alt = ui->altitudeSpinBox->value();

    if (alt >= 0) {
        int rows = ui->routesTableWidget->rowCount();
        addPoint(lat, longt, alt, rows);
    }

    return;
}

void Routes::deleteRow()
{
    QPushButton *button = qobject_cast<QPushButton*>(sender());
    if (button != NULL) {
        int row = ui->routesTableWidget->indexAt(button->pos()).row();
        ui->routesTableWidget->removeRow(row);
    }
    updateSectionsOrder();
}

void Routes::on_resetButton_clicked()
{
    ui->routesTableWidget->clearContents();
    ui->routesTableWidget->setRowCount(0);
    emit(reset());
    updateSectionsOrder();
}


void Routes::on_setRouteButton_clicked()
{
    updateSectionsOrder();
    emit(loadPoint(waypoints));
    waypoints.clear();
}

void Routes::updateSectionsOrder()
{
    waypoints.clear();
    ui->routesTableWidget->verticalHeader();
    double lat = 0;
    double longt = 0;
    double alt = 0;
    std::vector<int> positions;
    for(int i = 0; i < ui->routesTableWidget->rowCount(); i++)
    {
        positions.push_back(ui->routesTableWidget->verticalHeader()->logicalIndex(i));
        //qDebug() << positions[i];
    }

    for (int i = 0; i < positions.size(); i++) {
        lat = ui->routesTableWidget->item(positions[i], 0)->text().toDouble();
        longt = ui->routesTableWidget->item(positions[i], 1)->text().toDouble();
        alt = ui->routesTableWidget->item(positions[i], 2)->text().toDouble();
        waypoints.push_back({lat, longt, alt});
        //qDebug() << waypoints[i];
    }

    ui->routesTableWidget->clearContents();
    ui->routesTableWidget->setRowCount(0);
    for (int i = ui->mapWidget->countItems() - 1; i > 1; i--) {
        //qDebug() << ui->mapWidget->countItems();
        ui->mapWidget->removeItem(ui->mapWidget->getItem(i));               //БАГ С ОТОБРАЖЕНИЕМ АЙТЕМОВ
    }

    for (int i = 0; i < positions.size(); i++) {
        addPoint(waypoints[i][0], waypoints[i][1], waypoints[i][2], i);
        //qDebug() << ui->routesTableWidget->rowCount();
    }
}

void Routes::pressHandler(QPointF projPos)
{
    qDebug() << "PRESSED";
    QGV::GeoPos geoPos = ui->mapWidget->getProjection()->projToGeo(projPos);

    int rows = ui->routesTableWidget->rowCount();
    double alt = 0;
    if (rows > 0) {
        alt = ui->routesTableWidget->item(rows - 1, 2)->text().toDouble();
    } else {
        alt = 50;
    }
    addPoint(geoPos.latitude(), geoPos.longitude(), alt, rows);
}

void Routes::loadMap()
{
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
        QGV::GeoRect(QGV::GeoPos(55.76, 37.68), QGV::GeoPos(55.77, 37.69))));

    placemark = new Placemark(QGV::GeoPos(55.76618001715375, 37.6850103977006));
    placemark->setLat(55.76618001715375);
    placemark->setLongt(37.6850103977006);
    ui->mapWidget->addItem(placemark);

    //QObject::connect(ui->mapWidget)
    ui->mapWidget->setMouseTracking(true);
    QObject::connect(ui->mapWidget, &QGVMap::mapMousePress, this, &Routes::pressHandler);
}

void Routes::addPoint(double lat, double longt, double alt, int rows)
{
    ui->routesTableWidget->insertRow(rows);

    ui->routesTableWidget->setItem(rows, 0, new QTableWidgetItem(QString::number(lat, 'f', 6)));
    ui->routesTableWidget->setItem(rows, 1, new QTableWidgetItem(QString::number(longt, 'f', 6)));
    ui->routesTableWidget->setItem(rows, 2, new QTableWidgetItem(QString::number(alt, 'f', 0)));

    QPushButton *deleteButton = new QPushButton();
    deleteButton->setIcon(QIcon("/home/kolsun/ControlSystemGUI/images/deleteIcon.png"));
    deleteButton->connect(deleteButton, &QPushButton::clicked, this, &Routes::deleteRow);
    ui->routesTableWidget->setCellWidget(rows, 3, deleteButton);

    ui->routesTableWidget->item(rows, 0)->setTextAlignment(Qt::AlignVCenter | Qt::AlignHCenter);
    ui->routesTableWidget->item(rows, 1)->setTextAlignment(Qt::AlignVCenter | Qt::AlignHCenter);
    ui->routesTableWidget->item(rows, 2)->setTextAlignment(Qt::AlignVCenter | Qt::AlignHCenter);

    double prevLat = 0;
    double prevLongt = 0;
    if (rows > 0) {
        prevLat = ui->routesTableWidget->item(rows - 1, 0)->text().toDouble();
        prevLongt = ui->routesTableWidget->item(rows - 1, 1)->text().toDouble();
    } else {
        prevLat = placemark->getLat();
        prevLongt = placemark->getLongt();
    }

    QPointF projStart = ui->mapWidget->getProjection()->geoToProj(QGV::GeoPos(prevLat, prevLongt));
    QPointF projStop = ui->mapWidget->getProjection()->geoToProj(QGV::GeoPos(lat, longt));

    RouteLine *routeLine = new RouteLine(projStart, projStop, QColor(Qt::blue));
    ui->mapWidget->addItem(routeLine);
    Pointmark *point = new Pointmark(QGV::GeoPos(lat, longt));
    ui->mapWidget->addItem(point);

    if (rows == 1) {
        QObject::connect(this, &Routes::pointChanged, routeLine, &RouteLine::updatePoints);
    }
}

void Routes::reorder(int logicalIndex, int oldVisualIndex, int newVisualIndex)
{
    updateSectionsOrder();
}
