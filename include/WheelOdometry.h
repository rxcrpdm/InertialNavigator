#ifndef WHEELODOMETRY_H
#define WHEELODOMETRY_H

#include <QObject>

class WheelOdometry : public QObject
{
    Q_OBJECT

public:
    explicit WheelOdometry(QObject* parent = nullptr);

    void setWheelRadius(double r);
    void setWheelBase(double L);

    void updateEncoders(double leftTicks, double rightTicks, double dt);

    double forwardVelocity() const { return m_v; }
    double yawRate() const { return m_yawRate; }

signals:
    void odomUpdated(double v, double yawRate);

private:
    double m_radius;     // wheel radius (m)
    double m_base;       // wheel separation (m)

    double m_prevLeft;
    double m_prevRight;
    bool   m_first;

    double m_v;          // forward velocity
    double m_yawRate;    // yaw rate
};

#endif // WHEELODOMETRY_H

