#ifndef AIOBJECT_H
#define AIOBJECT_H

#define ENVSIZE 400
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
    float pitchAngleController(int);
    float rollAngleController(int);
    void pop_environment();
    QList<int> scan(int, int, QString);

    float error;
    int scan_N;
    int environment[ENVSIZE][ENVSIZE], cur_x, cur_y, prev_x, prev_y, dest_x, dest_y, arm_len;
    float kp, ki, kd, da;
    double angle, angle2;
    float prevError, integral, prevError2, integral2;
};

#endif // AIOBJECT_H
