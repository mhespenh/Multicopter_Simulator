/***********************************************************************
  Group 3 ECE4574 Spring 2012
  Project 3 - Multicopter Simulator
    Multicopter_Simulator - Control process for simulator (master branch)

  Revisions:
    03/21/2012 - Initial version (master branch)
    03/23/2012 - Implemented dbus communication correctly
***********************************************************************/

#include <QObject>
#include <QProcess>
#include <QtDBus/QtDBus>
#include <QSharedMemory>
#include "aiobject.h"

#define PI 3.1415926535897932384626433832795028841971693993751058209

struct data {
    int t0, t1, t2, t3;
    int cur_x, cur_y, target_x, target_y;
    double pitch, roll, altitude, v_x, v_y, v_z;
};

class MulticopterSimulator : public QObject
{
    Q_OBJECT
public:
    explicit MulticopterSimulator(int numProcs, QObject *parent = 0);
    ~MulticopterSimulator();
    void setMass(float);
    void setGravity(float);
    void setArmLength(float);

signals:

public slots: //these must be public to the dbus can hit them
    void processExit(int,QProcess::ExitStatus);
    void recvMessage(QString);
    void recvUpdate(int, double);
    void processStarted(QString);
    void setTargetPosition(int, int);
    void getAngles(void);

private slots:
    void updatePhysics(void);
    void writeSharedMem(void);

private:
    void updatePosition(void);
    void sendDbusMessage(QString, int);
    void sendAngleUpdate(double , double , double);
    double curPitch, curRoll, curAltitude, dt;
    double targetPitch, targetRoll, targetAltitude;
    double v_x, v_y, v_z; //velocity in the x and y plane
    double cur_x, cur_y, prev_x, prev_y, prev_alt; //positions
    int target_x, target_y, numMotors; //number of motors
    double* throttles; //pointer to throttle values
    float mass, gravity, armLength; //things
    int procCount;
    QProcess* proc;
    QProcess* procs;
    QDBusConnection bus;
    QSharedMemory sharedMem;
    AIObject theAI;
};
