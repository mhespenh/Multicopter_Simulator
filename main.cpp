/***********************************************************************
  Group 3 ECE4574 Spring 2012
  Project 3 - Multicopter Simulator
    Multicopter_Simulator - Control process for simulator (master branch)

  Revisions:
    03/21/2012 - Initial version (master branch)
***********************************************************************/

#include <QtCore/QCoreApplication>
#include <MulticopterSimulator.h>
#include <QDebug>

int main(int argc, char *argv[])
{
    QCoreApplication a(argc, argv);

    MulticopterSimulator sim;
    sim.writeData("Hello from another process\n");
    return a.exec();
}
