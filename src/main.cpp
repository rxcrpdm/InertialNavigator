#include <QCoreApplication>
#include "InertialNavigator.h"

int main(int argc, char *argv[])
{
    QCoreApplication app(argc, argv);

    InertialNavigator navigator;

    QObject::connect(&navigator, &InertialNavigator::stateUpdated,
                     [&](QQuaternion q, QVector3D v, QVector3D p)
    {
        qDebug() << "Orientation (quat):" << q;
        qDebug() << "Velocity:" << v;
        qDebug() << "Position:" << p << "\n";
    });

    navigator.start();

    // In a real system, IMU callbacks update these continuously:
    // navigator.setAccelerometer(accelVector);
    // navigator.setGyroscope(gyroVector);

    return app.exec();
}

