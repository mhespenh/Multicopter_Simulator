/***********************************************************************
  Group 3 ECE4574 Spring 2012
  Project 3 - Multicopter Simulator
    Multicopter_Simulator - Control process for simulator (master branch)

  Revisions:
    03/21/2012 - Initial version (master branch)
    03/23/2012 - Implemented dbus communication
***********************************************************************/

#include <QObject>
#include <QProcess>

class MulticopterSimulator : public QObject
{
    Q_OBJECT
public:
    explicit MulticopterSimulator(QObject *parent = 0);
    void writeData(QByteArray data);
    bool initDbus();

signals:

public slots:
    void updateError(void);
    void updateText(void);
    void updateExit(int,QProcess::ExitStatus);

private:
    QProcess* proc;
};
