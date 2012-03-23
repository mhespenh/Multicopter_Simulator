/***********************************************************************
  Group 3 ECE4574 Spring 2012
  Project 3 - Multicopter Simulator
    Multicopter_Simulator - Control process for simulator (master branch)

  Revisions:
    03/21/2012 - Initial version (master branch)
    03/23/2012 - Implemented dbus communication
***********************************************************************/

#include <QtCore/QCoreApplication>
#include <MulticopterSimulator.h>
#include <QDebug>
#include <QtDBus/QtDBus>
#include <QProcess>

int main(int argc, char *argv[])
{
    QCoreApplication a(argc, argv);
    MulticopterSimulator sim;
    if( sim.initDbus() ) {
        return a.exec();
    }
    else {
        return -1;
    }
}
