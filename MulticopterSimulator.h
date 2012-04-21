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

struct data {
    int t0, t1, t2, t3;
};

class MulticopterSimulator : public QObject
{
    Q_OBJECT
public:
    explicit MulticopterSimulator(int numProcs, QObject *parent = 0);
    ~MulticopterSimulator();

signals:

public slots:
    void processError(void);
    void processSTDOUT(void);
    void processExit(int,QProcess::ExitStatus);
    void recvMessage(QString);
    void recvUpdate(int, double);
    void processStarted(QString);

private slots:
    void updatePhysics(void);

private:
    void sendDbusMessage(QString, int);
    void sendAngleUpdate(double , double , double);
    bool writeSharedMem();
    double curPitch, curRoll, curAltitude, dt;
    double targetPitch, targetRoll, targetAltitude;
    int cur_x, cur_y, numMotors;
    double* throttles;
    QProcess* proc;
    QProcess* procs;
    QDBusConnection bus;
    QSharedMemory sharedMem;
};
