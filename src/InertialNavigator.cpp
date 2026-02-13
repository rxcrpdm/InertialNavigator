#include "InertialNavigator.h"
#include <QtMath>

InertialNavigator::InertialNavigator(QObject *parent)
    : QObject(parent),
      m_orientation(QQuaternion::fromEulerAngles(0,0,0)),
      m_velocity(0,0,0),
      m_position(0,0,0),
      m_dt(0.01) // 100 Hz INS update
{
    connect(&m_timer, &QTimer::timeout,
            this, &InertialNavigator::updateState);
}

void InertialNavigator::start()
{
    m_timer.start(int(m_dt * 1000.0));
}

void InertialNavigator::stop()
{
    m_timer.stop();
}

void InertialNavigator::setAccelerometer(const QVector3D &accel)
{
    m_accel = accel;
}

void InertialNavigator::setGyroscope(const QVector3D &gyro)
{
    m_gyro = gyro;
}

/* ===============================
   Orientation Update (Gyro Integration)
   =============================== */
QQuaternion InertialNavigator::integrateGyro(const QQuaternion &q,
                                             const QVector3D &gyro,
                                             double dt)
{
    // Convert angular velocity to quaternion derivative
    QQuaternion omegaQuat(0,
                          gyro.x(),
                          gyro.y(),
                          gyro.z());

    QQuaternion qDot = 0.5f * omegaQuat * q;
    QQuaternion newQ = q + qDot * dt;

    return newQ.normalized();
}

/* ===============================
   Rotate a vector from body → world
   =============================== */
QVector3D InertialNavigator::rotateVector(const QQuaternion &q,
                                          const QVector3D &v)
{
    QQuaternion vq(0, v.x(), v.y(), v.z());
    QQuaternion rq = q * vq * q.conjugate();
    return QVector3D(rq.x(), rq.y(), rq.z());
}

/* ===============================
   Complementary Filter (Roll/Pitch)
   =============================== */
void InertialNavigator::applyComplementaryFilter(QQuaternion &q,
                                                 const QVector3D &accel)
{
    float alpha = 0.02f;  // accelerometer weighting

    // Normalize accel vector
    QVector3D a = accel.normalized();

    // Roll & pitch from accelerometer
    float roll  = qAtan2(a.y(), a.z());
    float pitch = qAtan2(-a.x(), qSqrt(a.y()*a.y() + a.z()*a.z()));

    // Yaw comes from gyro only → keep existing yaw
    QVector3D euler = q.toEulerAngles();
    float yaw = qDegreesToRadians(euler.z());

    QQuaternion accQ = QQuaternion::fromEulerAngles(
                qRadiansToDegrees(pitch),
                qRadiansToDegrees(roll),
                qRadiansToDegrees(yaw));

    q = QQuaternion::slerp(q, accQ, alpha).normalized();
}

/* ===============================
   Main INS Update Loop
   =============================== */
void InertialNavigator::updateState()
{
    // 1. Integrate orientation using gyro
    m_orientation = integrateGyro(m_orientation, m_gyro, m_dt);

    // 2. Fuse accel for roll/pitch stabilization
    applyComplementaryFilter(m_orientation, m_accel);

    // 3. Convert accelerometer to world frame
    QVector3D accWorld = rotateVector(m_orientation, m_accel);

    // 4. Compensate gravity
    accWorld -= QVector3D(0, 0, 9.80665f);

    // 5. Integrate velocity (world frame)
    m_velocity += accWorld * m_dt;

    // 6. Integrate position
    m_position += m_velocity * m_dt;

    emit stateUpdated(m_orientation, m_velocity, m_position);
}

