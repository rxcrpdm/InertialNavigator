#ifndef INERTIALNAVIGATOR_H
#define INERTIALNAVIGATOR_H

#include <QObject>
#include <QVector3D>
#include <QQuaternion>
#include <QTimer>

class InertialNavigator : public QObject
{
    Q_OBJECT

public:
    explicit InertialNavigator(QObject *parent = nullptr);

    void setAccelerometer(const QVector3D &accel);   // ax, ay, az
    void setGyroscope(const QVector3D &gyro);        // wx, wy, wz (rad/s)
    void start();
    void stop();

    QQuaternion orientation() const { return m_orientation; }
    QVector3D velocity() const { return m_velocity; }
    QVector3D position() const { return m_position; }

signals:
    void stateUpdated(QQuaternion orientation,
                      QVector3D velocity,
                      QVector3D position);

private slots:
    void updateState();

private:
    // Raw IMU measurements
    QVector3D m_accel;  
    QVector3D m_gyro;   

    // INS states
    QQuaternion m_orientation;  // rotation body→world
    QVector3D m_velocity;       // world frame
    QVector3D m_position;       // world frame

    // Timing
    QTimer m_timer;
    double m_dt;    // integration timestep

    // Internal helper functions
    QQuaternion integrateGyro(const QQuaternion &q,
                              const QVector3D &gyro,
                              double dt);

    QVector3D rotateVector(const QQuaternion &q,
                           const QVector3D &v);

    void applyComplementaryFilter(QQuaternion &q,
                                  const QVector3D &accel);
};

#endif // INERTIALNAVIGATOR_H

