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
    AIObject(int, int, int);
    bool setDestination(int, int);
    void getTargetAngles(float&, float&, int, int);
private:
    void pop_environment();
    QList<int> scan(int, int, QString);

    int scan_N;
    int environment[600][600], cur_x, cur_y, prev_x, prev_y, arm_len, dest_x, dest_y;
    int stop;
};

#endif // AIOBJECT_H
