#ifndef ESKF_FUSION_H
#define ESKF_FUSION_H

#include <QObject>
#include <QVector3D>
#include <QQuaternion>
#include <Eigen/Dense>
using namespace Eigen;

class EskfFusion : public QObject
{
    Q_OBJECT

public:
    explicit EskfFusion(QObject* parent = nullptr);

    void setIMU(const QVector3D& accel, const QVector3D& gyro, double dt);
    void setWheelOdom(double v_meas, double yawRate_meas);

    QVector3D position() const { return m_p; }
    QVector3D velocity() const { return m_v; }
    QQuaternion orientation() const { return m_q; }

signals:
    void fusedState(QVector3D p, QVector3D v, QQuaternion q);

private:
    // Nominal state
    QVector3D m_p;   // position
    QVector3D m_v;   // velocity
    QQuaternion m_q; // orientation

    // Covariance
    Matrix<double, 9, 9> P;

    // Process noise
    Matrix<double, 6, 6> Qc;

    // Latest IMU
    QVector3D a_m;
    QVector3D w_m;

    // Wheel odom measurements
    double w_v;
    double w_yawRate;
    bool odomAvailable;

    // Helpers
    QVector3D rotate(const QVector3D& v);
    void propagate(double dt);
    void updateWheel();
};

#endif // ESKF_FUSION_H

