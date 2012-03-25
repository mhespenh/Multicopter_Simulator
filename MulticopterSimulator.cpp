/***********************************************************************
  Group 3 ECE4574 Spring 2012
  Project 3 - Multicopter Simulator
    Multicopter_Simulator - Control process for simulator (master branch)

  Revisions:
    03/21/2012 - Initial version (master branch)
    03/23/2012 - Implemented dbus communication correctly
***********************************************************************/

#include "MulticopterSimulator.h"
#include <QDebug>

MulticopterSimulator::MulticopterSimulator(QObject *parent) :
    QObject(parent), bus(QDBusConnection::sessionBus()), sharedMem("PRIVATE_SHARED")
{
 //For some reason it seems we can't connect on the dbus if we start the process here...?
 /*   proc = new QProcess(this);
    QObject::connect(proc, SIGNAL(readyReadStandardError()), this, SLOT(updateError()));
    QObject::connect(proc, SIGNAL(readyReadStandardOutput()), this, SLOT(updateText()));
    QObject::connect(proc, SIGNAL(finished(int,QProcess::ExitStatus)), this, SLOT(updateExit(int,QProcess::ExitStatus)));
    proc->start("/home/mhespenh/Desktop/Project2/SimMotor/SimMotor");

    proc->waitForStarted(-1); //wait for process to start
    qDebug() << "Proc started";
*/
    bus.connect("", "/", "edu.vt.ece.ack", "ack", this, SLOT(recvMessage(QString)));
    sendDbusMessage("Hello, world!", 69);
    writeSharedMem();
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

void MulticopterSimulator::recvMessage(QString msg) {
    qDebug() << "Received: " << msg;
}

void MulticopterSimulator::sendDbusMessage(QString msg, int type) {
    QDBusMessage out = QDBusMessage::createSignal("/", "edu.vt.ece.msg", "msg");
    out << msg << type;
    bus.send(out);
}

bool MulticopterSimulator::writeSharedMem() {
    if (sharedMem.isAttached()) {
        sharedMem.detach();
    }
    data* theData;

     if (!sharedMem.create(sizeof(data))) {
         return false;
     }
     sharedMem.lock();
     theData = (data*)sharedMem.data();
     theData->t0 = 55;
     theData->t1 = 6;
     theData->t2 = 71;
     theData->t3 = 8;
     sharedMem.unlock();
     return true;
}
