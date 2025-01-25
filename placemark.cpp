#include "placemark.h"

Placemark::Placemark(const QGV::GeoPos& geoPos)
{
    setFlag(QGV::ItemFlag::IgnoreScale);
    setFlag(QGV::ItemFlag::IgnoreAzimuth);
    setFlag(QGV::ItemFlag::Highlightable);
    setFlag(QGV::ItemFlag::HighlightCustom);
    setFlag(QGV::ItemFlag::Highlightable);
    setFlag(QGV::ItemFlag::Transformed);
    setGeometry(geoPos, QSize(64, 64), QPoint(16, 32));

    const QString url = "https://cdn-icons-png.flaticon.com/512/8598/8598680.png";
    // /home/kolsun/ControlSystemGUI/img/droneIcon.png
    load(url);
}

QTransform Placemark::projTransform() const
{
    return isFlag(QGV::ItemFlag::Highlighted) ? QGV::createTransfromScale(projAnchor(), 1.2) : QTransform();
}

void Placemark::projOnFlags()
{
    setOpacity(isFlag(QGV::ItemFlag::Highlighted) ? 0.3 : 1.0);
}

double Placemark::getLat()
{
    return mLat;
}

double Placemark::getLongt()
{
    return mLongt;
}

void Placemark::setLat(double lat)
{
    mLat = lat;
}

void Placemark::setLongt(double longt)
{
    mLongt = longt;
}
