#include "EskfFusion.h"
#include <QtMath>

EskfFusion::EskfFusion(QObject* parent)
    : QObject(parent),
      m_p(0,0,0),
      m_v(0,0,0),
      m_q(1,0,0,0),
      a_m(0,0,0),
      w_m(0,0,0),
      w_v(0),
      w_yawRate(0),
      odomAvailable(false)
{
    P.setIdentity();
    P *= 0.01;

    // Process noise (acc + gyro)
    Qc.setZero();
    Qc.block<3,3>(0,0) = 0.1 * Matrix3d::Identity();
    Qc.block<3,3>(3,3) = 0.05 * Matrix3d::Identity();
}

QVector3D EskfFusion::rotate(const QVector3D& v)
{
    QQuaternion vq(0, v.x(), v.y(), v.z());
    QQuaternion rq = m_q * vq * m_q.conjugate();
    return QVector3D(rq.x(), rq.y(), rq.z());
}

void EskfFusion::setIMU(const QVector3D &accel,
                        const QVector3D &gyro,
                        double dt)
{
    a_m = accel;
    w_m = gyro;

    propagate(dt);

    if (odomAvailable)
        updateWheel();
}

void EskfFusion::setWheelOdom(double v_meas, double yawRate_meas)
{
    w_v = v_meas;
    w_yawRate = yawRate_meas;
    odomAvailable = true;
}

/* ================================
   PROPAGATION STEP (Nominal + ESKF)
   ================================ */
void EskfFusion::propagate(double dt)
{
    QVector3D accWorld = rotate(a_m);
    accWorld -= QVector3D(0,0,9.80665);

    m_v += accWorld * dt;
    m_p += m_v * dt;

    QQuaternion omega(0, w_m.x(), w_m.y(), w_m.z());
    QQuaternion qDot = 0.5f * omega * m_q;
    m_q = (m_q + qDot * dt).normalized();

    // Error-state propagation Jacobian
    Matrix<double,9,9> F = Matrix<double,9,9>::Zero();
    F.block<3,3>(0,3) = Matrix3d::Identity();
    F.block<3,3>(3,6) = -Matrix3d::Identity(); // approx

    Matrix<double,9,9> Phi = Matrix<double,9,9>::Identity() + F * dt;

    Matrix<double,9,6> G = Matrix<double,9,6>::Zero();
    G.block<3,3>(3,0) = Matrix3d::Identity();
    G.block<3,3>(6,3) = Matrix3d::Identity();

    // Covariance update
    P = Phi * P * Phi.transpose() + G * Qc * G.transpose() * dt;
}

/* ================================
   WHEEL ODOMETRY UPDATE (ESKF)
   ================================ */
void EskfFusion::updateWheel()
{
    float heading = m_q.toEulerAngles().z() * M_PI / 180.0;

    double v_pred = m_v.x()*qCos(heading) + m_v.y()*qSin(heading);
    double yawRate_pred = w_m.z();

    Vector2d z;
    z << w_v, w_yawRate;

    Vector2d h;
    h << v_pred, yawRate_pred;

    Vector2d r = z - h;

    Matrix<double,2,9> H = Matrix<double,2,9>::Zero();
    H(0,3) = qCos(heading);
    H(0,4) = qSin(heading);
    H(1,8) = 1.0;

    Matrix2d R = Matrix2d::Identity() * 0.01;

    Matrix2d S = H * P * H.transpose() + R;
    Matrix<double,9,2> K = P * H.transpose() * S.inverse();

    Matrix<double,9,1> dx = K * r;

    m_p += QVector3D(dx(0), dx(1), dx(2));
    m_v += QVector3D(dx(3), dx(4), dx(5));

    QQuaternion dq = QQuaternion::fromEulerAngles(
                dx(6)*180.0/M_PI,
                dx(7)*180.0/M_PI,
                dx(8)*180.0/M_PI
    );

    m_q = (dq * m_q).normalized();

    P = (Matrix<double,9,9>::Identity() - K * H) * P;

    emit fusedState(m_p, m_v, m_q);
}

