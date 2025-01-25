#ifndef PLACEMARK_H
#define PLACEMARK_H

#include <QGeoView/QGVImage.h>

class Placemark : public QGVImage
{
    Q_OBJECT

public:
    explicit Placemark(const QGV::GeoPos& geoPos);

private:
    QTransform projTransform() const override;
    void projOnFlags() override;

public:
    double getLat();
    double getLongt();
    void setLat(double lat);
    void setLongt(double longt);

private:
    double mLat;
    double mLongt;
};

#endif
