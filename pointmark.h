#ifndef POINTMARK_H
#define POINTMARK_H

#include <QGeoView/QGVImage.h>

class Pointmark : public QGVImage
{
    Q_OBJECT

public:
    explicit Pointmark(const QGV::GeoPos& geoPos);

private:
    QTransform projTransform() const override;
    void projOnFlags() override;
};

#endif
