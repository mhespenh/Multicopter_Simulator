/***********************************************************************
  Group 3 ECE4574 Spring 2012
  Project 3 - Multicopter Simulator
    Multicopter_Simulator - Control process for simulator (master branch)

  Revisions:
    03/21/2012 - Initial version (master branch)
    03/23/2012 - Implemented dbus communication
***********************************************************************/

#include "MulticopterSimulator.h"
#include <QDebug>
#include <QtDBus/QtDBus>

MulticopterSimulator::MulticopterSimulator(QObject *parent) :
    QObject(parent)
{
/* //For some reason it seems we can't connect on the dbus if we start the process here...?
    proc = new QProcess(this);
    QObject::connect(proc, SIGNAL(readyReadStandardError()), this, SLOT(updateError()));
    QObject::connect(proc, SIGNAL(readyReadStandardOutput()), this, SLOT(updateText()));
    QObject::connect(proc, SIGNAL(finished(int,QProcess::ExitStatus)), this, SLOT(updateExit(int,QProcess::ExitStatus)));
    proc->start("/home/mhespenh/Desktop/Project2/SimMotor/SimMotor");

    proc->waitForStarted(10000); //wait for process to start
    qDebug() << "Proc started";
*/
}

void MulticopterSimulator::updateError() {
    QByteArray data = proc->readAllStandardError();
    qDebug() << QString(data);

}

void MulticopterSimulator::updateExit(int foo, QProcess::ExitStatus bar) {
    qDebug() << "Exited\n" << foo << "\t" << bar;
}

void MulticopterSimulator::updateText() {
    QByteArray data = proc->readAllStandardOutput();
    qDebug() << QString(data);
}

void MulticopterSimulator::writeData(QByteArray data) {
    proc->write(data);
}

bool MulticopterSimulator::initDbus() {
    if (!QDBusConnection::sessionBus().isConnected()) {
         fprintf(stderr, "Cannot connect to the D-Bus session bus.\n"
                 "To start it, run:\n"
                 "\teval `dbus-launch --auto-syntax`\n");
         return false;
     }

    QDBusInterface iface("edu.vt.ece.simmotor", "/", "", QDBusConnection::sessionBus());
     if (iface.isValid()) {
         QDBusReply<QString> reply = iface.call("recvMessage", "Hello", 45);
         if (reply.isValid()) {
             printf("Reply was: %s\n", qPrintable(reply.value()));
         }
         else {
            fprintf(stderr, "Call failed: %s\n", qPrintable(reply.error().message()));
        }
     }
     else {
         fprintf(stderr, "%s\n", qPrintable(QDBusConnection::sessionBus().lastError().message()));
         return false;
     }
     return true;
}
