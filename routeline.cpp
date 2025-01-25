#include "routeline.h"
#include "qpainter.h"

RouteLine::RouteLine(const QPointF projStart, const QPointF projStop, const QColor color)
    : mProjStart(projStart)
    , mProjStop(projStop)
    , mColor(color)
{
}

void RouteLine::updatePoints(const QGV::GeoPos geoPos_start, const QGV::GeoPos geoPos_stop)
{
    mProjStart = getMap()->getProjection()->geoToProj(geoPos_start);
    mProjStop = getMap()->getProjection()->geoToProj(geoPos_stop);
    resetBoundary();
    refresh();
}

QPainterPath RouteLine::projShape() const
{
    QPainterPath path;

    auto scale = 3 / getMap()->getCamera().scale();

    qreal dx = mProjStart.x() - mProjStop.x();
    qreal dy = mProjStart.y() - mProjStop.y();
    qreal length = sqrt(pow(dx,2) + pow(dy,2));

    path.moveTo(mProjStart.x(), mProjStart.y());
    path.lineTo(mProjStart.x() - dy/length*scale, mProjStart.y() + dx/length*scale);
    path.lineTo(mProjStop.x() - dy/length*scale, mProjStop.y() + dx/length*scale);
    path.lineTo(mProjStop.x(), mProjStop.y());
    path.closeSubpath();

    return path;
}

void RouteLine::projPaint(QPainter* painter)
{
    /*
    painter->setBrush(mColor);
    painter->drawPath(projShape());
*/
    auto scale = 3 / getMap()->getCamera().scale();
    painter->setPen(QPen(mColor, scale));
    painter->setBrush(mColor);
    painter->drawLine(mProjStart.x(), mProjStart.y(),mProjStop.x(), mProjStop.y());
}
