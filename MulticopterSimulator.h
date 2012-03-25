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

signals:

public slots:
    void updateError(void);
    void updateText(void);
    void updateExit(int,QProcess::ExitStatus);
    void recvMessage(QString);

private:
    void sendDbusMessage(QString, int);
    bool writeSharedMem();
    QProcess* proc;
    QDBusConnection bus;
    QSharedMemory sharedMem;
};
