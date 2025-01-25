#ifndef ROUTELINE_H
#define ROUTELINE_H

#include <QGeoView/QGVDrawItem.h>


class RouteLine : public QGVDrawItem
{
    Q_OBJECT

public:
    explicit RouteLine(const QPointF projStart, const QPointF projStop, const QColor color);

private:
    QPainterPath projShape() const override;
    void projPaint(QPainter* painter) override;
public slots:
    void updatePoints(const QGV::GeoPos geoPos_start, const QGV::GeoPos geoPos_stop);

private:
    QPointF mProjStart;
    QPointF mProjStop;
    QColor mColor;
};

#endif // ROUTELINE_H
