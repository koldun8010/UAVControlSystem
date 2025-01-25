#include "pointmark.h"

Pointmark::Pointmark(const QGV::GeoPos& geoPos)
{
    setFlag(QGV::ItemFlag::IgnoreScale);
    setFlag(QGV::ItemFlag::IgnoreAzimuth);
    setFlag(QGV::ItemFlag::Highlightable);
    setFlag(QGV::ItemFlag::HighlightCustom);
    setFlag(QGV::ItemFlag::Highlightable);
    setFlag(QGV::ItemFlag::Transformed);
    setGeometry(geoPos, QSize(40, 40), QPoint(16, 32));

    //const QString url = "https://cdn-icons-png.flaticon.com/512/467/467257.png";
    const QString url = "https://cdn-icons-png.flaticon.com/256/664/664684.png";
    // /home/kolsun/ControlSystemGUI/img/droneIcon.png
    load(url);
}

QTransform Pointmark::projTransform() const
{
    return isFlag(QGV::ItemFlag::Highlighted) ? QGV::createTransfromScale(projAnchor(), 1.2) : QTransform();
}

void Pointmark::projOnFlags()
{
    setOpacity(isFlag(QGV::ItemFlag::Highlighted) ? 0.3 : 1.0);
}
