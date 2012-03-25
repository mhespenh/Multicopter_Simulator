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
    explicit MulticopterSimulator(QObject *parent = 0);
    ~MulticopterSimulator();

signals:

public slots:
    void processError(void);
    void processSTDOUT(void);
    void processExit(int,QProcess::ExitStatus);
    void recvMessage(QString);
    void processStarted(QString);

private:
    void sendDbusMessage(QString, int);
    bool writeSharedMem();
    QProcess* proc;
    QDBusConnection bus;
    QSharedMemory sharedMem;
};
