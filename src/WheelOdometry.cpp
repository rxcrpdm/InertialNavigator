#include "WheelOdometry.h"
#include <QtMath>

WheelOdometry::WheelOdometry(QObject *parent)
    : QObject(parent),
      m_radius(0.3),
      m_base(1.2),
      m_prevLeft(0),
      m_prevRight(0),
      m_first(true),
      m_v(0),
      m_yawRate(0)
{}

void WheelOdometry::setWheelRadius(double r) { m_radius = r; }
void WheelOdometry::setWheelBase(double L)  { m_base = L; }

void WheelOdometry::updateEncoders(double leftTicks,
                                   double rightTicks,
                                   double dt)
{
    if (m_first) {
        m_prevLeft  = leftTicks;
        m_prevRight = rightTicks;
        m_first = false;
        return;
    }

    double dL = (leftTicks  - m_prevLeft)  * m_radius;
    double dR = (rightTicks - m_prevRight) * m_radius;

    m_prevLeft  = leftTicks;
    m_prevRight = rightTicks;

    m_v = (dL + dR) / (2.0 * dt);
    m_yawRate = (dR - dL) / (m_base * dt);

    emit odomUpdated(m_v, m_yawRate);
}

