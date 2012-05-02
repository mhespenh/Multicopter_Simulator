#ifndef AIOBJECT_H
#define AIOBJECT_H

#define ENVSIZE 600
#define DEFAULT_PITCH 0.0
#define DEFAULT_ROLL 0.0

#include<QDebug>
#include<QList>
#include <math.h>
#include <QtCore/QCoreApplication>

class AIObject
{
public:
    AIObject();
    bool setDestination(int, int);
    void getTargetAngles(double&, double&, int, int);
    void setArmLength(double);


private:
    float angleController(int);
    void pop_environment();
    QList<int> scan(int, int, QString);

    float error;
    int scan_N;
    int environment[600][600], cur_x, cur_y, prev_x, prev_y, arm_len, dest_x, dest_y;
    float kp, ki, kd, da;
    double angle;
    float prevError, integral;
};

#endif // AIOBJECT_H
