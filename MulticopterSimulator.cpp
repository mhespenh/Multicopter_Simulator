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
    writeSharedMem();

    proc = new QProcess(this); //new qprocess to run a motor process
    QObject::connect(proc, SIGNAL(readyReadStandardError()), this, SLOT(processError()));
    QObject::connect(proc, SIGNAL(readyReadStandardOutput()), this, SLOT(processSTDOUT()));
    QObject::connect(proc, SIGNAL(finished(int,QProcess::ExitStatus)), this, SLOT(processExit(int,QProcess::ExitStatus)));
    proc->start("/home/mhespenh/Desktop/Project2/SimMotor/SimMotor");

    proc->waitForStarted(-1); //wait for process to start
    qDebug() << "Proc started";
    //Signal: procStart- Motor process will emit this signal over the dbus when is starts and connects
    bus.connect("", "/", "edu.vt.ece.procStart", "procStart", this, SLOT(processStarted(QString)));
    bus.connect("", "/", "edu.vt.ece.ack", "ack", this, SLOT(recvMessage(QString)));
}

MulticopterSimulator::~MulticopterSimulator() {
    qDebug() << "Cleaning up...";
    sharedMem.detach();
    sharedMem.deleteLater();
}

void MulticopterSimulator::processStarted(QString reply) {
    qDebug() << "Received (DBus): " << reply;
    sendDbusMessage("Hello, world!", 69);
}

void MulticopterSimulator::processError() {
    QByteArray data = proc->readAllStandardError();
    data.chop(1); //remove the \n
    qDebug() << "Received (STDERR): " << QString(data);

}

void MulticopterSimulator::processExit(int foo, QProcess::ExitStatus bar) {
    qDebug() << "Process Exited with status: " << bar;
    qDebug() << "So we'll quit too.";
    QCoreApplication::quit();
}

void MulticopterSimulator::processSTDOUT() {
    QByteArray data = proc->readAllStandardOutput();
    qDebug() << "Received (STDOUT): " << QString(data);
}

void MulticopterSimulator::recvMessage(QString msg) {
    qDebug() << "Received (DBus): " << msg;
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
     theData->t0 = 5;
     theData->t1 = 6;
     theData->t2 = 7;
     theData->t3 = 8;
     sharedMem.unlock();
     qDebug() << "Wrote to Shared Memory: " << theData->t0 << theData->t1 << theData->t2 << theData->t3;
     return true;
}
